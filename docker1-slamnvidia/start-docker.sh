docker run --rm -it --gpus all -e QT_X11_NO_MITSHM=1 -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --hostname dockerpc randaz81/r1slam:nvidia

