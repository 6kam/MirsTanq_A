# MIRS MG5 ROS 2 Package

mirs_mg5の標準的機能を備えたROS 2パッケージ（Docker対応版）

## 🐳 主な改変ポイント

このリポジトリは以下のリポジトリから派生し、**Docker環境で簡単に実行できるように改変**されています：
- [mirs240x/mirs_mg5](https://github.com/mirs240x/mirs_mg5)
- [mirs240x/mirs_msgs](https://github.com/mirs240x/mirs_msgs)
- [mirs240x/mirs_slam_navigation](https://github.com/mirs240x/mirs_slam_navigation)

### Docker化のメリット
- **環境構築が簡単**: ROS 2 Humbleの手動インストール不要
- **依存関係が自動解決**: rosdepによる自動インストール
- **再現性が高い**: どのマシンでも同じ環境で実行可能
- **クリーンな環境**: ホストシステムを汚さない

## 📋 システム要件

- Docker
- Docker Compose
- X11サーバー（GUI表示用）

## 🚀 クイックスタート

### 1. リポジトリのクローン

```bash
git clone https://github.com/6kam/MirsTanq_A.git
cd MirsTanq_A/mirsws
```

### 2. Docker イメージのビルド

```bash
docker compose build
```

初回ビルドには数分かかります。以下の処理が自動的に行われます：
- ROS 2 Humble Desktop のインストール
- Navigation2、SLAM Toolbox などの依存パッケージのインストール
- sllidar_ros2、micro-ROS Agent のクローンとビルド
- ワークスペースのビルド

### 3. コンテナの起動

```bash
xhost +local:docker  # X11転送を許可（初回のみ）
docker compose up -d
docker compose exec mirs bash
```

### 4. ROS 2 ノードの実行

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

## 📦 含まれるパッケージ

### mirs
メインパッケージ。以下の機能を提供：
- ESP32との通信（micro-ROS）
- オドメトリ計算とTF配信
- ロボットモデル（URDF）
- ナビゲーション設定
- SLAM設定

### mirs_msgs
カスタムメッセージ定義パッケージ

## 🔧 開発方法

### ソースコードの編集

ホストマシンの `src/` ディレクトリを編集すると、コンテナ内にも反映されます（ボリュームマウント）。

### ビルド

コンテナ内で：

```bash
cd /root/mirs/mirsws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` オプションにより、Pythonスクリプトやlaunchファイルの変更は再ビルド不要です。

### デバッグ

```bash
# ノードのリスト表示
ros2 node list

# トピックのリスト表示
ros2 topic list

# トピックのデータ確認
ros2 topic echo /cmd_vel

# TFツリーの確認
ros2 run tf2_tools view_frames
```

## 🔌 ハードウェア接続

### USB デバイス

`docker-compose.yml` で以下のデバイスがマウントされています：
- `/dev/ttyUSB0`: LiDAR または ESP32
- `/dev/ttyUSB1`: ESP32 または その他のデバイス

デバイスが異なる場合は、`docker-compose.yml` を編集してください。

### 権限の設定

USBデバイスへのアクセスには権限が必要な場合があります：

```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

## 📝 ライセンス

このプロジェクトはMITライセンスの下で公開されています。詳細は [LICENSE](LICENSE) を参照してください。

## 🙏 謝辞

このプロジェクトは以下のリポジトリから派生しています：
- [mirs240x/mirs_mg5](https://github.com/mirs240x/mirs_mg5)
- [mirs240x/mirs_msgs](https://github.com/mirs240x/mirs_msgs)
- [mirs240x/mirs_slam_navigation](https://github.com/mirs240x/mirs_slam_navigation)

元の開発者の皆様に感謝いたします。

## 📚 参考リンク

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Navigation2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [micro-ROS](https://micro.ros.org/)
