#### Trimming

```bash
ffmpeg -ss 00:04:08 -t 00:04:36 -i input_video.MOV -vcodec copy -acodec copy output_video.mp4
```

#### Conversion

```bash
ffmpeg -i input_video.mp4 -r 30 -vf "scale=720:-1,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" output_video.gif
```