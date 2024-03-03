# Author: Rolf Hettel, AVL Deutschland

import sys
import time
import numpy as np
from icos import ICOS_Connection
import keyboard


# PARAMETERS
SHOW_DEBUG_MSG = False       # If true, outputs some debug information. Deactivate to increase performance, True..False
TRACKED_KEYS_LIST = ["up",  # List of keys that will be tracked. Ports will be created automatically for each one
                     "down",
                     "left", 
                     "right"]  
FILTER_KP = 0.02
FILTER_KI = 0.001
KEYS_DE = {1: "esc",        # Key binding on a German keyboard
           80: "down",
           72: "up",
           75: "left",
           77: "right",
           73: "page up",
           81: "page down",
           28: "enter",
           82: "insert",
           83: "delete",
           71: "home",
           79: "end",
           69: "num lock",
           57: "space",
           14: "backspace",
           29: "ctrl",
           56: "alt",
           541: "alt gr",
           42: "shift",
           54: "right shift",
           15: "tab",
           58: "caps lock",
           91: "windows",
           92: "right windows",
           93: "menu",
           55: "print",
           70: "scroll",
           59: "f1",
           60: "f2",
           61: "f3",
           62: "f4",
           63: "f5",
           64: "f6",
           65: "f7",
           66: "f8",
           67: "f9",
           68: "f10",
           87: "f11",
           88: "f12",
           30: "a",
           48: "b",
           46: "c",
           32: "d",
           18: "e",
           33: "f",
           34: "g",
           35: "h",
           23: "i",
           36: "j",
           37: "k",
           38: "l",
           50: "m",
           49: "n",
           24: "o",
           25: "p",
           16: "q",
           19: "r",
           31: "s",
           20: "t",
           22: "u",
           47: "v",
           17: "w",
           45: "x",
           44: "y",
           21: "z",
           40: "ä",
           39: "ö",
           26: "ü",
           41: "^",
           2: "1",
           3: "2",
           4: "3",
           5: "4",
           6: "5",
           7: "6",
           8: "7",
           9: "8",
           10: "9",
           11: "0",
           12: "ß",
           51: ",",
           52: ".",
           53: "-",
           43: "#",
           27: "+",
           86: "<"}
pressed_key_codes = []   # Stores the currently pressed keys (as integer code)
pressed_key_names = []   # Stores the currently pressed keys (as string names)
tracked_key_values = []
error_integral = []
           
     
def pressed_keys(e):
    global pressed_key_codes 
    global pressed_key_names 
    pressed_key_codes = [code for code in keyboard._pressed_events]    
    pressed_key_names = [KEYS_DE[code] if (code in KEYS_DE) else "-" for code in keyboard._pressed_events]
    if ((len(pressed_key_codes) > 0) and (SHOW_DEBUG_MSG)):
        print("keys: ", pressed_key_names, pressed_key_codes)
    
    
def main(args):
    # CONNECTING TO MODEL.CONNECT
    try:
        icos = ICOS_Connection()
    except Exception as exc:
        sys.exit(-1)
    #icos.sendWarningMsg(0, " > Infotext!")
    #icos.sendWarningMsg(1, " > Warningtext!")
    #icos.sendWarningMsg(2, " > Errortext!")
    print("START")
    icos.sendWarningMsg(0, ">>> KEYBOARD CONTROL | Python interface started")
    icos.sendWarningMsg(0, ">>> KEYBOARD CONTROL | Arguments: " + str(args))
    sim_time = 0.0
    end_time  = icos.getEndTime()   
    icos.sendWarningMsg(0, ">>> KEYBOARD CONTROL | Endtime: " + str(end_time))    
    icos_params = icos.getModelDefsSpecial()  # Get ICOS parameters
    SIM_STEPSIZE = icos_params["TimeStep"]    
    
    # INITIALIZE
    tracked_key_values = np.zeros(len(TRACKED_KEYS_LIST))
    error_integral = np.zeros(len(TRACKED_KEYS_LIST))
    
    # RUN SIMULATION   
    icos.sendWarningMsg(0, ">>> KEYBOARD CONTROL | Running...")
    print("RUN")
    start_time = time.time()  
    set_keyboard_hook = True
    try:
        while (sim_time <= end_time):
            act_time = time.time() - start_time 
            sim_time += SIM_STEPSIZE
            if (sim_time >= end_time):
                print("SIMULATION END REACHED")
                break
                
            # Read inputs from M.C
            activate = icos.getScalarInput("activate", sim_time) 
            activate_output_filter = icos.getScalarInput("activate_output_filter", sim_time) 
            
            # Calculation
            active = 1 if (activate == 1) else 0
            if ((set_keyboard_hook) and (active == 1)):
                keyboard.hook(pressed_keys)
                set_keyboard_hook = False
                icos.sendWarningMsg(0, ">>> KEYBOARD CONTROL | Active")
            if (not(set_keyboard_hook) and (active == 0)):
                keyboard.unhook(pressed_keys)
                set_keyboard_hook = True
                icos.sendWarningMsg(0, ">>> KEYBOARD CONTROL | Inactive")
            
            # Write outputs to M.C        
            icos.postScalarOutput("active", sim_time, active)
            for i in range(len(TRACKED_KEYS_LIST)):
                key = TRACKED_KEYS_LIST[i]
                if (activate_output_filter == 1):  # Filter the output value if the key is pressed, using PI controller
                    error = int(key in pressed_key_names) - tracked_key_values[i]
                    error_integral[i] = error * SIM_STEPSIZE
                    tracked_key_values[i] += error * FILTER_KP + error_integral[i] * FILTER_KI
                else:   # Just output the actual state of the key
                    tracked_key_values[i] = int(key in pressed_key_names)
                icos.postScalarOutput("key_" + key, sim_time, tracked_key_values[i])        
    except:
        print("SIMULATION STOP ERROR")
        icos.sendWarningMsg(1, ">>> RTS CONTROL | Simulation stopped or error")
        #sys.exit(-1)    
    
    icos.sendWarningMsg(0, ">>> KEYBOARD CONTROL | Python interface stopped")
    print("EXIT")


if __name__ == "__main__":
    args = []
    if (len(sys.argv) > 1):
        args = sys.argv
    main(args)
