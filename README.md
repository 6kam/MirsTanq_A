# MIRS MG5 ROS 2 Package

mirs_mg5の標準的機能を備えたROS 2パッケージ（Docker対応版）

## 主な改変点

このリポジトリは以下のリポジトリから派生し、**Docker環境で簡単に実行できるように改変**されています：
- [mirs2502/mirs_mg5](https://github.com/mirs2502/mirs_mg5)
- [mirs2502/mirs_description](https://github.com/mirs2502/mirs_description)
- [mirs240x/mirs_mg5](https://github.com/mirs240x/mirs_mg5)
- [mirs240x/mirs_msgs](https://github.com/mirs240x/mirs_msgs)
- [mirs240x/mirs_slam_navigation](https://github.com/mirs240x/mirs_slam_navigation)

### Dockerを使用するメリット
- **環境構築が簡単**: ROS 2 Humbleと、lidarやmicro-ros-agentといったros2パッケージのインストール不要
- **依存関係が自動解決**: rosdepによる自動インストール
- **再現性が高い**: どのマシンでも同じ環境で実行可能
- **クリーンな環境**: ホストシステムを汚さない

## 要件

### ハードウェア

- cugo v3i
- ESP32
- PC
- Cytron MD10C

### ソフトウェア

- Docker (Docker Desktop for Windowsは使用不可)
- Docker Compose
- X11サーバー (windowsの場合WSLg)
- USBIPD-WIN https://github.com/dorssel/usbipd-win (windowsのみusb接続のために必要)
- Arduino IDE (ESP32にmicro-ros-clientを導入するため)

## 含まれるパッケージ

### mirs
メインパッケージ。以下の機能を提供：
- ESP32との通信（micro-ROS）
- オドメトリ計算とTF配信
- ロボットモデル（URDF）
- ナビゲーション
- SLAM

### mirs_msgs
カスタムメッセージ定義パッケージ

## つかいかた

### 1. ESP32のセットアップ

  Arduino IDEに以下のソースコードとライブラリを導入する
  - [mirs2502/mirs2502_esp32(esp32用ソースコード)](https://github.com/mirs2502/mirs2502_esp32.git)
    - 2502の元になったコード[mirs240x/mirs24_esp32(esp32用ソースコード)](https://github.com/mirs240x/mirs24_esp32.git)
  - [mirs240x/micro_ros_arduino_mirs240x](https://github.com/mirs240x/micro_ros_arduino_mirs240x)
    - micro-rosライブラリ。zipでインポート

### 2. リポジトリのクローン

```bash
git clone https://github.com/6kam/MirsTanq_A.git
cd MirsTanq_A/mirsws
```

### 3. Docker イメージのビルド

```bash
docker compose build
```

初回ビルドには数分かかります。以下の処理が自動的に行われます：
- ROS 2 Humble Desktop のインストール
- Navigation2、SLAM Toolbox などの依存パッケージのインストール
- sllidar_ros2、micro-ROS Agent のクローンとビルド
- ワークスペースのビルド

### 4. コンテナの起動

```bash
xhost +local:docker  # X11転送を許可（初回のみ）
docker compose up -d
docker compose exec mirs bash
```

### 5. ROS 2 ノードの実行

lidarとesp32をpcにusb接続すること
### USB デバイス

`docker-compose.yml` で以下のデバイスがマウントされています：
- `/dev/ttyUSB0`: LiDAR
- `/dev/ttyUSB1`: ESP32
先にlidarを接続することでttyUSB0がLiDARになるはずです。launchのときにusbポートを指定する必要はありません。

コンテナ内で：

```bash
# 基本的なシステム起動
ros2 launch mirs mirs.launch.py

# ロボットモデルの可視化（RViz2）
ros2 launch mirs display.launch.py

# SLAM実行
ros2 launch mirs pc_slam.launch.py

# ナビゲーション
ros2 launch mirs navigation.launch.py
```

## 開発方法

### ソースコードの編集

ホストマシンの `src/` はマウントされていないので、ホストで編集したら再度ビルドする必要があります。
コンテナ内での編集はホストに反映されません。

### デバッグ

```bash
# ノードのリスト表示
ros2 node list

# トピックのリスト表示
ros2 topic list

# トピックのデータ確認
ros2 topic echo /odom
ros2 topic echo /encoder #cugov3はクローラのため値は2つ出ます

# TFツリーの確認
ros2 run tf2_tools view_frames
```

## コンテナの終了方法
```bash
exit
docker compose down
```

## ライセンス

このプロジェクトはMITライセンスの下で公開されています。詳細は [LICENSE](LICENSE) を参照してください。

## 謝辞

このプロジェクトは以下のリポジトリから派生しています：
- [mirs2502/mirs_mg5](https://github.com/mirs2502/mirs_mg5)
- [mirs2502/mirs_description](https://github.com/mirs2502/mirs_description)
- [mirs240x/mirs_mg5](https://github.com/mirs240x/mirs_mg5)
- [mirs240x/mirs_msgs](https://github.com/mirs240x/mirs_msgs)
- [mirs240x/mirs_slam_navigation](https://github.com/mirs240x/mirs_slam_navigation)

開発者の皆様に感謝申し上げます。

## 参考リンク

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Navigation2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [micro-ROS](https://micro.ros.org/)
