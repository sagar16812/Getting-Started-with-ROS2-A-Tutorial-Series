# How To Convert ROS bag to ROS2 bag

Best way to convert rosbag file to ros2 compatible bag:

## Installation:
```bash
pip3 install rosbags>=0.9.11
```

## Usage:
```bash
rosbags-convert --src my_rosbag1.bag --dst ros2_bag_output
```

Options:
```bash
usage: rosbags-convert [-h] --src [PATH ...] --dst DST
                       [--dst-storage {sqlite3,mcap}]
                       [--dst-version DST_VERSION]
                       [--compress {none,bz2,lz4,zstd}]
                       [--compress-mode {file,message,storage}]
                       [--src-typestore {empty,latest,ros1_noetic,ros2_dashing,ros2_eloquent,ros2_foxy,ros2_galactic,ros2_humble,ros2_iron,ros2_jazzy} | --src-typestore-ref SRC_TYPESTORE_REF  --dst-typestore {copy,empty,latest,ros1_noetic,ros2_dashing,ros2_eloquent,ros2_foxy,ros2_galactic,ros2_humble,ros2_iron,ros2_jazzy} | --dst-typestore-ref DST_TYPESTORE_REF]
                       [--exclude-topic [TOPIC ...]]
                       [--include-topic [TOPIC ...]]
                       [--exclude-msgtype [MSGTYPE ...]]
                       [--include-msgtype [MSGTYPE ...]]
```

## To convert for ROS 2 Humble
For my ros2 humble, I use this command:
```bash
rosbags-convert --src my_rosbag1.bag --dst ros2_bag_output --dst-typestore ros2_humble
```