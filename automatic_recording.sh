#!/bin/bash

cd ~/shared/data/moving_background/ev-datasets/triangle/4dof/800

echo "create folder"
yarpdatadumper --name /data &
echo "data dumper"
sleep 2
yarp connect /atis3/AE:o /data &
echo "connection"
sleep 20
killall yarpdatadumper
yarp clean

