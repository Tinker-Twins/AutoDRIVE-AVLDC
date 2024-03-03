import socket
import struct
import sys
import time
import queue
from threading import Thread

from icos import ICOS_Connection


# PARAMETERS
SHOW_DEBUG_MSG = True  # If true, outputs some debug information. Deactivate to increase performance, True..False
VTD_IP = "10.19.192.132"  # IP address of the VTD machine
VTD_PORT = 48179  # port on the VTD machine sending the SCP messages
RECV_QUEUE_SIZE = 200  # maximum message queue size for received SCP (do not change)
FILTER = ["<Camera", "<TEST", "</TEST>", "<TEST/>"]  # filter words (case sensitive). Only output messages containing those. [""] --> allow all


def scp_reader(scp_socket: socket.socket, msg_queue: queue.Queue):
    """ Thread for reading SCP messages coming from VTD. Fills up message queue """
    while True:
        message = read_scp_message(scp_socket)[:-1]  # read new message from TCP port
        if (message != ""):
            if (not (msg_queue.full())):
                msg_queue.put(message)  # put new SCP message to message queue
            else:  # stop if scp message queue is full
                print("MSG QUEUE FULL - SCP READER THREAD STOPPED")
                break


def create_scp_socket(ip: str, port: int):
    """ Create TCP socket to receive SCP messages from VTD.
    Returns the socket object """
    scp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    scp_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    scp_socket.connect((ip, port))
    return scp_socket


def send_scp_message(scp_socket: socket.socket, message: str):
    """ Send a SCP message on the specified socket to VTD"""
    msg = message.encode("utf-8")
    fmt = "= H H 64s 64s I"
    magic_no = 40108
    version = 0x0001
    tool_name = b"Model.CONNECT"
    receiver = b"any"
    msg_header = struct.pack(fmt, magic_no, version, tool_name, receiver, len(msg))
    scp_socket.send(msg_header)
    scp_socket.send(msg)


def read_scp_message(scp_socket: socket):
    """ Read SCP message from TCP socket
    Returns the message as a string """
    h_packed = scp_socket.recv(136)  # get SCP message header
    header = struct.unpack("= H H 64s 64s I", h_packed)
    data_size = header[-1]  # get length of SCP message data
    data = scp_socket.recv(data_size)  # read SCP message data from TCP socket
    message = data.decode("utf-8")  # decode SCP message data to string
    return message


def filter_message(message, filter):
    """ Check if message contains on of the filter words.
    Returns true if msg contains filter, otherwise false """
    if ("" in filter):
        return True
    result = False
    for f in filter:  # check for all filter words
        if (f in message):  # if message contains the filter word
            result = True
            break
    return result


def main(args):
    # CONNECTING TO MODEL.CONNECT
    try:
        icos = ICOS_Connection()
    except Exception as exc:
        sys.exit(-1)
    # icos.sendWarningMsg(0, " > Infotext!")
    # icos.sendWarningMsg(1, " > Warningtext!")
    # icos.sendWarningMsg(2, " > Errortext!")
    print("START")
    icos.sendWarningMsg(0, ">>> SCP RECEIVE | Python interface started")
    icos.sendWarningMsg(0, ">>> SCP RECEIVE | Arguments: " + str(args))
    sim_time = 0.0
    end_time = icos.getEndTime()
    icos.sendWarningMsg(0, ">>> SCP RECEIVE | Endtime: " + str(end_time))
    icos_params = icos.getModelDefs()  # Get ICOS parameters
    SIM_STEPSIZE = icos_params["TimeStep"]

    # INITIALIZE
    msg_queue = queue.Queue(maxsize=RECV_QUEUE_SIZE)  # create thread-safe queue for scp messages
    scp_socket = create_scp_socket(ip=VTD_IP, port=VTD_PORT)  # create socket object
    p = Thread(target=scp_reader, args=(scp_socket, msg_queue))  # create thread for scp reader
    p.start()  # start scp reader thread
    last_filtered_message = ""
    send_trigger_latch = 0
    active = 0  # return value if model is active, 0 = no data from VTD or inactive, 1 = active
    active_timer = 0

    # RUN SIMULATION
    icos.sendWarningMsg(0, ">>> SCP RECEIVE | Running...")
    print("RUN")
    start_time = time.time()
    try:
        while (sim_time <= end_time):
            act_time = time.time() - start_time
            sim_time += SIM_STEPSIZE
            if (sim_time >= end_time):
                icos.sendWarningMsg(1, ">>> SCP RECEIVE | Simulation end reached...")
                print(round(sim_time, 2), "s | SIMULATION END REACHED")
                break

            # Read inputs from M.C
            #activate = icos.getScalarInput("Activate_In", sim_time)
            msg_send = icos.getStringInput("MSG_In", sim_time)
            msg_send_trigger = icos.getScalarInput("MSG_Send_Trigger_In", sim_time)

            # Calculation   
            if (not(msg_queue.empty())):
                active_timer = sim_time                   
            if (sim_time - active_timer > 1.0):
                active = 0
            else:
                active = 1
            if (SHOW_DEBUG_MSG):
                print(round(sim_time, 2), "s | MSG QUEUE SIZE:", msg_queue.qsize(), "/", RECV_QUEUE_SIZE)
            # read SCP messages from queue
            msg_received = 0  # return value if filtered message is received, 0 = not received, 1 = received from VTD
            if (msg_queue.full()):  # if received messages stacks up --> queue full
                icos.sendWarningMsg(1, ">>> SCP RECEIVE | SCP MESSAGE QUEUE FULL")
                print(round(sim_time, 2), "s | MSG QUEUE FULL - MAIN STOPPED")
                break
            while (True):  # repeat until no more messages are in message queue
                if (not (msg_queue.empty())):  # still messages on the queue
                    message = msg_queue.get()  # get message from queue
                    if (filter_message(message, FILTER)):  # check if message contains filter words
                        last_filtered_message = message
                        msg_received = 1
                        if (SHOW_DEBUG_MSG):
                            print(round(sim_time, 2), "s | MSG: ", message)
                else:
                    break  # if all SCP messages are read from queue                  
            
            # Send SCP messages
            if ((msg_send_trigger == 1) and (send_trigger_latch == 0)):
                send_scp_message(scp_socket, msg_send)
                send_trigger_latch = 1
            elif (msg_send_trigger == 0):
                send_trigger_latch = 0              

            # Write outputs to M.C
            icos.postScalarOutput("Active_Out", sim_time, active)
            icos.postScalarOutput("MSG_Received_Out", sim_time, msg_received)
            icos.postStringOutput("MSG_Out", sim_time, last_filtered_message)

    except:
        print("SIMULATION STOP ERROR")
        scp_socket.close()  # close TCP socket
        icos.sendWarningMsg(1, ">>> SCP RECEIVE | Simulation stopped or error")
        sys.exit(1)

    scp_socket.close()  # close TCP socket
    icos.sendWarningMsg(0, ">>> SCP RECEIVE | Python interface stopped")
    print("EXIT")


if __name__ == "__main__":
    args = []
    if (len(sys.argv) > 1):
        args = sys.argv
    main(args)
