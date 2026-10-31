# LiDAR Perception System

ROS 2 Humble向けの移動バケツ認識パッケージです。相手ロボット座標系に点群を蓄積し、バケツの開口中心と底面を推定します。

## ビルド

ROS 2 Humble、Eigen3、nlohmann_jsonが必要です。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_perception_system --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

旧版から更新する場合は、このパッケージの `build/lidar_perception_system` と `install/lidar_perception_system` を削除してから再ビルドしてください（通常の分離installの場合）。旧ノード・旧launchの残存を防ぎます。

## 起動前の準備

起動に使う各ターミナルで、次を実行してください。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

既定の入力は `/dynamic_cloud`、基準座標系は `map` です。入力点群が別座標系の場合は、点群時刻に対応する `map` へのTFも必要です。このパッケージのlaunchは、認識と鮮度監視を起動します。LiDARドライバ・シミュレータ・自己位置推定は別途起動してください。

以下の「RVizなし」「RVizあり」は、どちらか一方を実行します。「RVizのみ」は、認識がすでに起動しているときの表示用です。

## 実機で起動

先にLiDARドライバと自己位置推定を起動し、`/dynamic_cloud` と必要なTFが配信されていることを確認します。実機では通常の時刻を使います。

### RVizなし

```bash
ros2 launch lidar_perception_system bucket_tracker.launch.py
```

### RVizあり

```bash
ros2 launch lidar_perception_system bucket_tracker_rviz.launch.py
```

### 設定済みRVizのみ

認識を「RVizなし」で起動済みの場合に使います。

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix --share lidar_perception_system)/rviz/bucket_tracker.rviz"
```

## シミュレーション・rosbagで起動

点群・必要なTF・`/clock` を配信し、シミュレーション時刻を使います。認識の起動方法はシミュレーションとrosbagで共通です。

### RVizなし

```bash
ros2 launch lidar_perception_system bucket_tracker.launch.py use_sim_time:=true
```

### RVizあり

```bash
ros2 launch lidar_perception_system bucket_tracker_rviz.launch.py use_sim_time:=true
```

### 設定済みRVizのみ

認識を「RVizなし」で起動済みの場合に使います。RVizもシミュレーション時刻で起動します。

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix --share lidar_perception_system)/rviz/bucket_tracker.rviz" --ros-args -p use_sim_time:=true
```

## 設定

既定設定は [config/bucket_tracker.json](config/bucket_tracker.json) です。入力トピック、探索範囲 `opponent_roi`、フィールド境界 `field_bounds` を使用環境の `map` 座標に合わせてください。同梱設定の変更は再ビルドで反映します。

別のJSONを使う場合は、各launchに `config_file:=/absolute/path/to/config.json` を追加します。

## 出力

ノード・launch名は整理しましたが、既存のトピック名・メッセージ型・TF名は維持しています。

| トピック | 内容 |
|---|---|
| `/bucket_opening_tracker/target` | 鮮度監視後の開口中心（`MovingBucketTrack`） |
| `/bucket_opening_tracker/target_raw` | 鮮度監視前の開口推定 |
| `/bucket_opening_tracker/bottom` | 確定した底面形状（`BucketBottom`） |
| `/bucket_opening_tracker/bottom_track` | 平滑化した追跡・推定を含む底面位置（`BucketBottom`） |
| `/bucket_opening_tracker/bottom_track_raw` | 平滑化前の底面追跡位置（揺れの解析用） |
| `/bucket_opening_tracker/diagnostics` | 処理状態・TF待ち・処理時間など |
| `/bucket_opening_tracker/selected_cloud` | 選択した相手点群 |
| `/bucket_opening_tracker/accumulated_cloud` | 運動補償した蓄積点群 |

RVizでは点群と認識マーカーを確認できます。点群が見えない場合は、入力トピック、Fixed Frame（既定 `map`）、点群時刻のTFを確認してください。

出力を使う側では `valid`・観測時刻・`observation_mode` を確認してください。予測・推定結果と直接観測は区別し、底面位置を開口中心として使わないでください。`confidence` は校正された確率ではありません。実データでの連続認識・絶対精度は未受入です。
