# ping_pong_measurer_ros2

## TODO

- [x] ペイロードサイズ可変
- [x] 並列実行
- [x] 計測データファイル保存
- [x] 計測データ日時追加（秒以下を含む）
- [x] CPU、メモリ使用率計測ノード
- [x] 使用方法のアップデート

## Prerequisites

install followings,

* [ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
* [colcon](https://colcon.readthedocs.io/en/released/user/installation.html)
* `g++`

## Getting started

### Prepare measurement

```
$ mkdir -p ros2_ws/src
$ git clone https://github.com/b5g-ex/ping_pong_measurer_ros2.git ros2_ws/src/ping_pong_measurer
$ cd ros2_ws
$ ln -s src/ping_pong_measurer/Makefile .
```

### How to measure

#### on Pong machine

1. start pong node by `make run_pong` under `ros2_ws` directory.


#### on Ping machine

1. start ping node by `make run_ping` under `ros2_ws` directory.
2. start os_info node by `make os_info` under `ros2_ws` directory.
3. start starter node by `make starter` under `ros2_ws` directory.

* starter triggers measurement.
* data is stored under ros2_ws/data/
 
### changeable parameters

 We can change following parameters by modifying Makefile,

* --node-counts
  * we need to change both ping and pong node counts **SAME**.
* --payload-bytes
* --measurement-times

