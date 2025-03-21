# Low-latency Visual Servoing with Event-cameras

The algorithm tracks known shapes on a planar surface using an event camera. Specifically, it estimates the target center, its rotation around the center, and its scaling.  

It runs three modules in parallel contained in the /code folder:
- the EROS representation 
- Affine 4-DoF tracking 
- Robot controller

## 👨🏻‍💻 Datasets
Event-based datasets and their ground truth are available.

## ⚙ Build
Build the docker using:
```
cd four-dof-affine-tracking
docker build -t affine:latest - < Dockerfile
```

Make the container using:
```
docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb -v /tmp/.X11-unix/:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --network host --name affine affine:latest
```

Inside the container compile your code: 
```
docker exec -it affine bash
```
```
cd /usr/local/src/four-dof-affine-tracking/code/
mkdir build && cd build
cmake ..
make
make install
```

## ▶ Run
The algorithm works under [YARP](https://github.com/robotology/yarp) middleware. 

Run the server, typing on the terminal.
```
yarpserver
```
Load the event-driven dataset sample of the moving shape, by using the GUI opened after typing on the terminal:
```
yarpdataplayer
```
Follow _File>Open directory>Select_ and select the folder inside /usr/local/src/four-dof-affine-tracking/sample-dataset, containing data.log and info.log files. It should upload the data. 

Run the algorithm:
```
shape_position --eros_k 11 --eros_d 0.6 --shape_file /usr/local/src/four-dof-affine-tracking/shapes/star.png 
```
A picture should pop up showing the star template. 

Click the play button on yarpdataplayer to stream the events on the YARP port. It should show the EROS events in the picture.

Press 'g' on the picture to start tracking when the moving shape overlaps the template in the middle of the image.



