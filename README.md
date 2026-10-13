# LiDAR Perception System

ROS 2 HumbleとCUDAを使う、相手の移動バケツ専用のLiDAR認識パッケージです。入力はLivox Mid-360等の`PointCloud2`と時刻対応の`map → base_link → センサ` TFです。点群の変換と高さ別グリッド集計をGPUで行い、支持部の候補を追跡します。

固定バケツ・旗の認識、PCL、背景PCD、KD-tree背景差分、中間点群の配信は使用しません。既知の固定設備のTFがある場合は、その周辺を候補から除外します。

## ビルド

必要な直接依存: ROS 2 Humble、CUDAToolkit、Eigen3、nlohmann_json。Ceresは現実装では使用しません。CUDA GPUと互換ドライバが必要です。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_perception_system --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=89
source install/setup.bash
```

`CMAKE_CUDA_ARCHITECTURES`は対象機のGPUに合わせて指定してください。開発機のRTX 4060 Laptop GPUでは`89`を使用しました。Jetsonの型番を確認せずに`89`を流用しないでください。

## 実機で使う

先にMid-360のドライバと自己位置推定を起動し、`/livox/lidar`と点群時刻に対応する`map → センサフレーム`のTFが配信されていることを確認します。別のターミナルで以下を実行します。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch lidar_perception_system moving_bucket_rviz.launch.py
```

このlaunchは検出ノードとRViz2を**両方**起動します。検出だけを動かす場合は`moving_bucket.launch.py`を使用し、同じ検出ノードを二重起動しないでください。設定は[config/moving_bucket.json](config/moving_bucket.json)です。入力トピックや座標系が異なる場合は設定ファイルをコピーして変更し、`config_file:=/absolute/path/to/moving_bucket.json`をlaunch引数に渡します。

## 大会bagをRViz2で確認する

bagは`.db3`ファイル単体ではなく、`metadata.yaml`を含む**rosbagディレクトリ**を指定します。まずターミナル1で検出ノードとRViz2を起動します。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch lidar_perception_system moving_bucket_rviz.launch.py use_sim_time:=true bag_path:=/absolute/path/to/rosbag.db3
```

`bag_path`には再生する`.db3`ファイルまたはそれを含むディレクトリを指定します。このオプションは、途中再生で飛ばされる`/tf_static`を先に読み込んで配信します。`bag_path`なしで冒頭から再生する場合は、bag自身の`/tf_static`を使用できます。

ターミナル2で、同じbagを再生します。

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /absolute/path/to/rosbag.db3 --clock
```

`--clock`と`use_sim_time:=true`は必ず組み合わせてください。今回の決勝bagを試合開始（記録開始から15分36秒）へ飛ばす場合は、上の`ros2 bag play`コマンドに`--start-offset 936`を追加します。`bag_path`を省いて途中再生すると固定TFが失われ、RVizの点群が`Message Filter ... queue is full`で破棄されることがあります。

## RViz2画面の読み方

固定フレームは設定の`target_frame`と同じ`map`です。元点群の上に、目標位置の円柱、速度矢印、直近100観測の軌跡、観測状態の文字が表示されます。

- 緑 `DIRECT`: バケツ本体を直接観測。
- 橙 `SUPPORT / Z inferred`: 支持部から推定。Zは暫定値。
- 青 `PREDICTED`: 新しい観測がなく、短時間だけ位置を予測。
- 赤 `NO TARGET`: 有効な目標なし。位置マーカーは削除され、理由と`tf_drops`を表示。

RVizのDisplaysパネルで`PointCloud2 (Livox)`、`Moving Bucket`、必要に応じて`TF (optional)`の表示を切り替えられます。別のターミナルで以下のコマンドを個別に実行してトピックも確認できます。`ros2 topic hz`はCtrl+Cで終了してください。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic hz /livox/lidar
ros2 topic echo /moving_bucket_detector/target --once
ros2 topic echo /moving_bucket_detector/diagnostics --once
```

`target.valid=true`かつ`observation_mode`が`DIRECT`または`SUPPORT_INFERRED`なら観測による追跡です。`PREDICTED`は観測ではありません。`valid=false`の古い位置を照準に使用しないでください。

点群が表示されない場合は、入力トピック、RVizのFixed Frame、`map → センサフレーム`のTF、bag再生時刻を確認します。動的TF欠落中に生点群だけを見るには、RVizのFixed Frameを一時的に`livox_frame`へ変更できます（その間、map座標の目標マーカーは表示できません）。点群が見えるのに赤い`NO TARGET`が続く場合は、対象が検出条件に入っていない可能性があります。`tf_drops`が増え続ける場合は、点群の時刻に対応するTFが不足しています。状態表示まで消えた場合は検出ノードと入力トピックが動作しているか確認してください。

マーカーはRViz等の購読者がいるときだけ生成します。RVizで位置が見えることは精度保証ではなく、点群との重なりや別途用意した正解データで妥当性を評価してください。

追跡済みの支持部候補は短い点群欠測後も速度と位置から再関連付けします。ただし、`PREDICTED`は観測ではなく、既定では最後の実観測から0.3秒までです。TFが長く欠落する、対象がLiDARから隠れるなど新しい観測を得られない間は`valid=false`になります。表示を途切れさせないためだけに予測時間を長くするのは避け、まず自己位置推定の`map → base_link` TFと点群時刻を確認してください。診断の`support_candidate_frames`と`support_association_frames`はそれぞれ支持部候補があった処理フレーム数と支持部仮説が候補を返したフレーム数です（最終追跡での採用件数とは異なります）。

## 出力

- `/moving_bucket_detector/target`: `lidar_perception_system/msg/MovingBucketTrack`。位置、速度、共分散、観測状態、validを含みます。
- `/moving_bucket_detector/diagnostics`: 入力件数、処理件数、TF待ち超過、キュー破棄、処理異常、処理時間のp95・最大値、TF待ち時間のp95。
- `map → moving_bucket` TF: 設定で有効化した場合、直接観測時のみ配信します。

`observation_mode=SUPPORT_INFERRED`はバケツ本体が直接見えていないことを表します。現在の`support_target_z=1.2 m`は大会bagによる暫定値であり、実物の支持部と開口中心の関係を実測するまで、Zを高精度な観測値として扱わないでください。`confidence`は校正された確率ではありません。`valid=false`の場合は古い位置を照準に使わないでください。

## 検証

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/LiDAR-Perception-System
python3 scripts/benchmark_cuda.py /path/to/rosbag_directory --offset 0 --seconds 180 --output /tmp/lps_benchmark.json
```

ベンチマークにはrosbagディレクトリまたは`.db3`ファイルを直接指定できます。`--offset`は記録開始からの秒数です。`--include-traces`を付けると全座標履歴を出力するため、生成ファイルは機密の大会データとして扱ってください。比較対象の`/opponent_robot/bucket_target`は既存アルゴリズムの結果で、正解ラベルではありません。精度の合否には、映像や実測位置に基づく別の正解データが必要です。精度を評価するときは観測不能な区間とTF欠落区間を分けて集計してください。
