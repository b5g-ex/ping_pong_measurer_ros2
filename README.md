# ping_pong_measurer_ros2

## Prerequisites

install followings,

* [ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
* [colcon](https://colcon.readthedocs.io/en/released/user/installation.html)
* `g++`


## Prepare measurement

```
$ mkdir -p ros2_ws/src
$ git clone https://github.com/b5g-ex/ping_pong_measurer_ros2.git ros2_ws/src/ping_pong_measurer
$ cd ros2_ws
$ ln -s src/ping_pong_measurer/Makefile .
```

## How to measure

### on Pong machine

1. start pong node by `make run_pong` under `ros2_ws` directory.
2. confirm all node starts up by following command which returns node counts.  
   `$ ros2 node list | grep pong | wc -l`  
   this is necessary because starting nodes takes a time.

### on Ping machine

1. start os_info node by `make os_info` under `ros2_ws` directory.
2. start ping node by `make run_ping` under `ros2_ws` directory.
3. confirm all node starts up by following command which returns node counts.  
   `$ ros2 node list | grep pong | wc -l`  
   this is necessary because starting nodes takes a time.
4. start starter node by `make starter` under `ros2_ws` directory.  
   starter triggers measurement.  
   data is stored under `ros2_ws/data/`
 
### changeable parameters

We can change following parameters by modifying Makefile,

* --node-counts
  * we need to change both ping and pong node counts **SAME**.
* --payload-bytes
* --measurement-times

