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

同梱の底面半径上限は、Unityで使用するバケツCADの底面外周半径約107.6mmに余裕を加えた115mmです。
別形状のバケツでは `bottom.min_radius`・`bottom.max_radius` を実寸に合わせてください。

底面の追跡位置・TF・マーカーには、速度予測を使う平滑化を適用します。
`bottom.output_smoothing_s` は既定 `0.15` 秒で、`0` にすると位置・法線の平滑化を無効にできます。
平滑化は短周期の揺れを抑えますが、急な加減速には応答遅れが生じます。
対象変更・対象追跡のリセット期限（`tracking.reset_s`）を超える欠落・時刻巻き戻りではフィルタをリセットします。
確認済み形状の大きな高さ変化では、高さの履歴のみをリセットします。
観測欠落中の出力の鮮度条件は変えず、同じ対象を追跡できる間はフィルタ内部の状態だけを保持します。
単発の大きな位置補正は制限し、複数フレームの観測で追従します。
直近0.5秒の観測から低速・停止と判断した間は平滑化を強め、微小な速度予測の残りを抑えます。
この間は低速移動の応答も遅くなります。`output_smoothing_s: 0` ではこの処理も無効になります。
水平の追跡位置は速度・加速度と整合する軌道として出力します。
`bottom.output_max_acceleration_mps2`（既定 `6.0`）で出力の水平加速度上限を指定します。
上限を小さくすると補正が緩やかになり、急な加減速・旋回への追従も遅くなります。
停止を推定した間はXY位置の仮説を保持し、速度または累積移動量の観測で保持を解除します。
`bottom.output_stationary_radius_m`（既定 `0.025`）は直接認識に基づく保持解除の移動量のしきい値です。
ごく低速の動き始めはこの距離まで表示が遅れる場合があり、`0` で停止保持を無効にできます。
底面を見ている履歴と車体から補完した履歴は分けて扱います。補完時は位置補正の重みを下げ、
短期・長期の動きが整合する場合に移動速度を反映します。直接認識がない間は移動開始の検出や位置補正が遅れる場合があります。
高さの変更ではXYの平滑化を初期化しません。
高さはXYと分けて扱い、側面だけの追跡では学習済みの高さを維持します。
`bottom.horizontal_only` は既定 `true` です。移動バケツが常に水平である制約を使い、
底面の当てはめ・追跡で法線をmapの鉛直方向 `(0, 0, 1)` に固定します。
底面高さと水平位置は点群から推定し、バケツの昇降には追従します。
`max_tilt_rad` による傾斜推定は `horizontal_only: false` の場合に使用します。
Z方向の速度予測は行わず、底面の平面・形状を観測した場合に高さを更新します。
少数点による平面追跡は既存平面の距離許容幅内に限定し、大きな昇降は底面形状の再認識で確認します。
`bucket_bottom` TFのZ軸は底面法線、X軸はmapのX方向を底面に投影した向きです。
円形底面のヨー角は観測できないため、このTFの向きを車体の向きとして使わないでください。

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
