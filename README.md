# mirs_mg5
mirs_mg5の標準的機能を備えたROS 2パッケージ

## Attention!
launchはyamlファイルへのパスを自動捜索するように書いているが環境によっては見つからずにエラーが起きる模様。\
cloneした後、それぞれ絶対パスに書き換えることを推奨。

書き換えるのはそれぞれのlaunchファイル内の
```
os.path.join(get_package_share_directory('mirs'),'config','various_file_names.yaml')
```

## Installation

```bash
$ cd ~/your_ws/src
$ git clone https://github.com/mirs240x/mirs_mg5.git
$ cd ~/your_ws
$ colcon build --symlink-install
$ source ~/your_ws/install/setup.bash
```
micro-rosのセットアップも必要。

## 起動コマンド一覧

### 基本的な起動方法

#### 1. フルシステム起動（推奨）
LIDAR、カメラ、コーン検出、自律ナビゲーション、ビヘイビアツリーをすべて起動：
```bash
./launch_with_log.sh mirs system_bringup.launch.py
```
または
```bash
ros2 launch mirs system_bringup.launch.py
```

#### 2. ロボット本体 + LIDAR起動
```bash
ros2 launch mirs mirs.launch.py serial_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0
```
ポートは環境に応じて変更してください。デバイスの確認：
```bash
ls /dev/ttyUSB*
```

#### 3. オドメトリのみ（外部位置推定なし）
```bash
ros2 launch mirs mirs_odom_only.launch.py
```

### サブシステム個別起動

#### コーン検出パイプライン
```bash
ros2 launch cone_detector cone_detection.launch.py
```

#### ビヘイビアツリーミッション実行
```bash
ros2 launch bt_pkg real_mission.launch.py
```

#### 経路生成（矩形パターン）
```bash
ros2 run coverage_planner rectangle_generator
```

### ビジュアライゼーション

#### RViz2起動
```bash
rviz2
```

#### Groot起動（ビヘイビアツリー可視化）
```bash
Groot
# ポート2666でbt_executorに接続
```

### デバッグ用

#### トピック確認
```bash
# 利用可能なトピック一覧
ros2 topic list

# 特定トピックの内容表示
ros2 topic echo /coverage_path
ros2 topic echo /cone_area
ros2 topic echo /cmd_vel
```

#### ノード確認
```bash
# 起動中のノード一覧
ros2 node list

# 特定ノードの情報
ros2 node info /bt_main
```

#### ログ確認
```bash
# ログディレクトリ
cd ~/mirs_ws/logs

# 最新ログ表示
tail -f $(ls -t ~/mirs_ws/logs/*.log | head -1)
```

### パラメータ調整

主要な設定ファイル：
- ロボットパラメータ: `src/mirs_mg5/mirs/config/config.yaml`
- ナビゲーション: `src/mirs_mg5/mirs/config/nav2_params.yaml`
- EKF設定: `src/mirs_mg5/mirs/config/ekf_params.yaml`
- ビヘイビアツリー: `src/bt_pkg/behavior_tree/bt_mission.xml`
- 経路生成: `coverage_planner/rectangle_generator.py` (safety_margin: 0.75m)

### トラブルシューティング

#### ビルドエラー
```bash
# クリーンビルド
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

#### デバイスが見つからない
```bash
# シリアルポート権限
sudo chmod 666 /dev/ttyUSB*

# USB接続確認
lsusb
```

#### Grootが落ちる
- Behavior Treeの構造が複雑すぎる可能性
- ビジュアライゼーション不要なら無視してOK（ロボット動作には影響なし）
