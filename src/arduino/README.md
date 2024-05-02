## micro-ROS Agent
```shell
docker run -it --rm --privileged -v /dev:/dev -v /dev/shm:/dev/shm --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -v6
```