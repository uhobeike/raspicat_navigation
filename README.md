# raspicat_navigation
[![ros-melodic](https://github.com/uhobeike/raspicat_navigation/actions/workflows/raspicat_navigation-ci.yaml/badge.svg)](https://github.com/uhobeike/raspicat_navigation/actions/workflows/raspicat_navigation-ci.yaml)

Raspberry Pi CatでROSの[Navigation Stack](https://wiki.ros.org/navigation)を使ってナビゲーションをするためのパッケージです。

地図の生成には[raspicat_slam](https://github.com/rt-net/raspicat_slam)にある`slam_remote_pc.launch`を使うことができます。

[Raspberry Pi Cat](https://rt-net.jp/products/raspberry-pi-cat/)を用いた[実機でのナビゲーションの動作確認](https://youtu.be/ObsD6C73Xr4)をしています。

### つくばチャレンジでの走行(動画)

[![Image from Gyazo](https://i.gyazo.com/144f8de089ce6e8d32e23afc78d66bf6.png)](https://youtu.be/TeF1nFxjLRQ)

## 実機
### 動作環境
#### Raspberry Pi Cat
##### ハードウェア

* [Raspberry Pi Cat](https://rt-net.jp/products/raspberry-pi-cat/)
  * with Raspberry Pi 3B, B+
  * with LiDAR
    * USB
      * URG-04LX-UG01
      * UTM-30LX
    * Ether
      * UST-10LX
      * UST-20LX

##### ソフトウェア

* OS
  * [Ubuntu 'classic' 18.04 LTS](https://wiki.ubuntu.com/ARM/RaspberryPi)
* Device Driver
  * [rt-net/raspicat_setup_scripts](https://github.com/rt-net/raspicat_setup_scripts)
* ROS
  * [Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
* ROS Packages
  * [rt-net/raspicat_ros](https://github.com/rt-net/raspicat_ros)
  * [rt-net/raspicat_gamepad_controller](https://github.com/rt-net/raspicat_gamepad_controller)
  * [rt-net/raspicat_slam](https://github.com/rt-net/raspicat_slam)
  * [uhobeike/raspicat_navigation](https://github.com/uhobeike/raspicat_navigation)

#### Remote PC

##### ソフトウェア
* OS
  * [Ubuntu 18.04 LTS](https://www.ubuntulinux.jp/News/ubuntu1804-ja-remix)
* ROS
  * [Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
* ROS Packages
  * [rt-net/raspicat_slam](https://github.com/rt-net/raspicat_slam)
  * [uhobeike/raspicat_navigation](https://github.com/uhobeike/raspicat_navigation)

### インストール

##### Raspberry Pi Cat
```sh
# ROSパッケージのダウンロード
cd ~/catkin_ws/src
git clone https://github.com/rt-net/raspicat_ros.git
git clone https://github.com/rt-net/raspicat_gamepad_controller.git
git clone https://github.com/rt-net/raspicat_slam.git
git clone https://github.com/uhobeike/raspicat_navigation.git

# 依存パッケージのインストール
rosdep install -r -y -i --from-paths .

# make & install
cd ~/catkin_ws && catkin build
source devel/setup.bash
```

##### Remote PC
```sh
# ROSパッケージのダウンロード
cd ~/catkin_ws/src
git clone https://github.com/rt-net/raspicat_slam.git
git clone https://github.com/uhobeike/raspicat_navigation.git

# 依存パッケージのインストール
rosdep install -r -y -i --from-paths .

# make & install
cd ~/catkin_ws && catkin build
source devel/setup.bash
```

### 使い方

#### SLAM

##### Raspberry Pi Cat

Raspberry Pi Cat上で以下のコマンドでノードを起動します。

端末でRaspberry Pi Cat を操作する[gamepad_controller](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.940-000144.html)及び、LiDARのためのlaunchを起動します。

(imuがない場合は`false`、ある場合は`true`を設定します。)

* when USB URG is connected e.g.) [URG-04LX-UG01](https://www.hokuyo-aut.co.jp/search/single.php?serial=17), [UTM-30LX](https://www.hokuyo-aut.co.jp/search/single.php?serial=21)
```sh
roslaunch raspicat_slam slam_remote_robot_usb_urg.launch imu:=false
```

* when Ether URG is connected e.g.) [UST-10LX](https://www.hokuyo-aut.co.jp/search/single.php?serial=16), [UST-20LX](https://www.hokuyo-aut.co.jp/search/single.php?serial=16)

```sh
roslaunch raspicat_slam slam_remote_robot_ether_urg.launch ip_address:="192.168.0.10" imu:=false
```

##### Remote PC

Remote PC上で以下のコマンドでノードを起動します。

1枚目の端末で地図を作成するためのlaunchを起動します。

```sh
roslaunch raspicat_slam slam_remote_pc.launch
```

地図作成後、2枚目の端末で地図を保存します。

```sh
roscd raspicat_navigation/config/maps
rosrun map_server map_saver -f mymap
```

#### Navigation

##### Raspberry Pi Cat

Raspberry Pi Cat上で次のコマンドでノードを起動します。

```sh
roslaunch raspicat_navigation raspicat_bringup.launch
```

##### Remote PC

Remote PC上で次のコマンドでノードを起動します。

```sh
roslaunch raspicat_navigation raspicat_navigation.launch 
```

## ライセンス

このリポジトリはApache License 2.0ライセンスで公開されています。詳細は[LICENSE](./LICENSE)を確認してください。

