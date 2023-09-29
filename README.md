# affine2dtracking

Build the docker using:

```
cd affine2dtracking
docker build -t affine:latest - < Dockerfile
```

Make the container using:

```
docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb -v /tmp/.X11-unix/:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --network host --name affine affine:latest
```

