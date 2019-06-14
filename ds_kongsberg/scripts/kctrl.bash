#!/bin/bash
#
# Requires docker and the kctrl docker image downloaded.
# Download kctrl.tar.gz from the dav
#
# Use these instructions once for setup:
# $ sudo apt-get update
# $ sudo apt-get install docker
# $ sudo docker load < kctrl.tar.gz
#
# Allow docker to be run by the current user without sudo:
# $ sudo groupadd docker
# $ sudo usermod -aG docker $USER

echo "Starting K Controller from a launch file..."
docker run --rm -it --network host kctrl