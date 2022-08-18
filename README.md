# ping_pong_measurer_ros2

## TODO

- [x] ペイロードサイズ可変
- [x] 並列実行
- [ ] 計測データファイル保存
- [ ] CPU、メモリ使用率計測ノード

## Prerequisites

* ROS2 foxy
* `colcon`

## Getting started

### Prepare measurement

```
$ mkdir -p ros2_ws/src
$ git clone https://github.com/b5g-ex/ping_pong_measurer_ros2.git ros2_ws/src/ping_pong_measurer
$ cd ros2_ws
$ ln -s src/ping_pong_measurer/Makefile .
```

### How to measure

1. start pong node by `make run_pong` under `ros2_ws` directory.
2. start ping node by `make run_ping` under `ros2_ws` directory.
