# Development notes

The main work so far with this branch has been to find a way to make the Raspberry Pi 5 and Camera
module v3 compatible with ROS.

The Raspberry Pi Camera module v3 uses a new libary referred to as [libcamera](https://libcamera.org/)
this is quite a major change from the previous raspberry pi cameras, see software description 
[here](https://www.raspberrypi.com/documentation/computers/camera_software.html).

I was unable to find a ros1 package that supported the new camera stack, however there is the node
[camera_ros](https://github.com/christianrauch/camera_ros). That is compatible with ROS2, it also 
made sense to switch to ROS2 for learning purposes as the structure and commands have changed 
significantly since ROS1.

It was still my intention to run ROS2 within docker and the most considerable challenge was 
finding a way to pass the camera hardware resource through to docker so it can be used by the
container. Simply running the docker container as 'privileged' was not enough. This took many hours 
to solve, most of the time I was stuck with camera_ros unable to detect any cameras at all (v2 or v3).

## Passing the camera resource to the docker image

I finally found a solution from this example repo [PiCamera2-ROS2-Humble-Docker](https://github.com/nagtsnegge/PiCamera2-ROS2-Humble-Docker)
but even this repo needed a bit of work, as it turns out it only functions for a fairly old version of
libcamera. Luckily there was a pull-request for an update submitted only a few days prior to me finding
the repo that solved this problem. 

This therefore results in the following weird configuration setup:
- libcamera commit used at version `6ddd79b`
- ros humble being used rather than jazzy (which was the latest at the time of writing), due to the libcamera version only working with Ubuntu jammy
- camera_ros being built from source rather than installing with APT, due to it needing to reference the specific version of libcamera

Within the dockerfile most of the packages installed are simply to support the compilation of libcamera
from source.

## noVNC setup

To aid with the development and testing of the ExoMy robot several ROS2 graphical tools will be used. 
To allow these to be displayed whilst still using docker the noVNC docker image needs to be used, the
configuration of noVNC was largely based on this guide
[here](https://divyanshu-raj.medium.com/ros-2-with-docker-part-1-9060f3095811).

The short summary of using noVNC is as follows:

1. Install docker as per the original ExoMy instructions.
1. Create a docker network using the command `docker network create ros`
1. As this is running on the raspberry pi there wasn't an `ARM64` build available at thime, so it had to be built from source. Clone this repo [here](https://github.com/theasp/docker-novnc) into the home directory `git clone https://github.com/theasp/docker-novnc`
1. CD into the folder and build the image tagging it as *novncamd* using the following command `docker build -t novncamd .`
1. Once it has built, the image can then be run each time using the following command:
    ```bash
    docker run -d --rm \
    --net=ros \
    --env="DISPLAY_WIDTH=1024" \
    --env="DISPLAY_HEIGHT=768" \
    --env="RUN_XTERM=no" \
    --name=novnc \
    -p=8081:8081 \
    novncamd
    ```

To connect to noVNC you can then simlpy open up a browser either on the Raspberry Pi or on another 
computer on the netowrk. 

As you can see from above, the port 8081 is used. So in my case the raspberry pi is on the home network
with local IP address `192.168.1.159`. I therefore opened a browser and navigated to 
http://192.168.1.159:8081/vnc.html to open up the screen. Once you click connect, anything that 
generates a GUI window on the shall then appear in the browser. 

The necessary environment variable and network setup have already been added to the `docker run`
command to enable it to properly connect to the noVNC network.

## Development tools (image_view, rviz etc.)

### Image View

To verify that the camera and ROS 2 connection is working correctly, I often use the following:

1. Launch noVNC using the multiline command above.
1. Launch the exomy dev using the command `sh docker/run_exomy.sh -d`
1. Start the camera node by using the command `ros2 run camera_ros camera_node`. You will know it's
successful when you see the camera being identified and the calibration used. If it says something
like *Camera Not Found* there are issues either with the camera or the flow through to Docker. First
test with the raspberry pi direct, to rule out any hardware issues.
1. Next open a second docker terminal connected to the same image. In the new terminal run `docker
container ls` to determine the Container ID of the exomy image.
1. Then launch a second bash terminal to the running container by typing `docker exec -it ???? bash`
where `????` are the first 4 characters of the Container ID. Note you just need to use enough
characters to uniquely identify the container, but 4 normally does the job.
1. You will then need to source the ros libraries again using `source install/setup.bash`
1. Using ROS2 see what topics are active using `ros2 topic list`. You will likely see
`/camera/image_raw` amongst others.
1. Finally I then execute image_view mapping the raw image to the tool using
`ros2 run image_view image_view --ros-args --remap /image:=/camera_ros/image_raw`
1. Then opening a browser to the port 8081 for the Raspberry Pi should then produce a live preview of
the camera output.

## Known bugs/Future Work

- [ ] Missing camera calibration likely due to needing raspberry pi files as well
- [ ] Webserver needs to be updated and built for ROS2 using an npm compatible with os
- [ ] Controller mapping needs to be fixed
- [ ] Webserver to be updated with camera_ros
- [ ] GUI packages and mapping to noVNC only for development
- [ ] Create a docker compose or something to automatically launch noVNC when 
- [ ] Update libcamera to latest and make compatible with docker