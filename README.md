# LiDAR Perception System

ROS 2 Humbleを使う、相手の移動バケツ専用のLiDAR認識パッケージです。従来のCUDA支持部追跡に加えて、CPUで相手座標系に点群を蓄積して開口中心を推定する初期実装を含みます。以下の従来方式と、新方式の専用launchは別々に起動できます。

従来方式はCUDAを使います。入力はLivox Mid-360等の`PointCloud2`と時刻対応の`map → base_link → センサ` TFです。点群の変換と高さ別グリッド集計をGPUで行い、支持部の候補を追跡します。

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

GPUなしで新しい開口追跡ノード・鮮度監視ノード・CPU試験をビルドする場合は、`--cmake-args -DBUILD_GPU_NODES=OFF`を指定します。ROS型とCPUの`support_tracker_test`だけをビルドする場合は`-DBUILD_RUNTIME_NODES=OFF`を指定できます。

## 実機で使う

先にMid-360のドライバと自己位置推定を起動し、`/livox/lidar`と点群時刻に対応する`map → センサフレーム`のTFが配信されていることを確認します。別のターミナルで以下を実行します。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch lidar_perception_system moving_bucket_rviz.launch.py
```

このlaunchは検出ノード・鮮度監視ノード・RViz2を起動します。RViz2なしで検出・鮮度監視を動かす場合は`moving_bucket.launch.py`を使用し、同じ検出ノードを二重起動しないでください。設定は[config/moving_bucket.json](config/moving_bucket.json)です。入力トピックや座標系が異なる場合は設定ファイルをコピーして変更し、`config_file:=/absolute/path/to/moving_bucket.json`をlaunch引数に渡します。

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

- `/moving_bucket_detector/target_raw`: GPU検出ノードが公開する監視前の追跡結果。
- `/moving_bucket_detector/target`: `lidar_perception_system/msg/MovingBucketTrack`。位置、速度、共分散、観測状態、validを含みます。
- `/moving_bucket_detector/diagnostics`: 入力件数、処理件数、TF待ち超過、キュー破棄、処理異常、処理時間のp95・最大値、TF待ち時間のp95。
- `map → moving_bucket` TF: 設定で有効化した場合、直接観測時のみ配信します。

`observation_mode=SUPPORT_INFERRED`はバケツ本体が直接見えていないことを表します。現在の`support_target_z=1.2 m`は大会bagによる暫定値であり、実物の支持部と開口中心の関係を実測するまで、Zを高精度な観測値として扱わないでください。`confidence`は校正された確率ではありません。`valid=false`の場合は古い位置を照準に使わないでください。

### 追跡結果の鮮度監視

CUDA検出ノードの`/moving_bucket_detector/target_raw`をCPUの`track_freshness_guard_node`が検査し、`/moving_bucket_detector/target`と任意の対象TFを配信します。`moving_bucket.launch.py`は両ノードを起動します。

入力点群・追跡結果・最終観測の時刻、受信からの単調期限、有限値・共分散・観測状態を確認します。GPU計算停止や入力断でも20ms周期で無効結果を公開し、古い結果や重複結果では期限を延長しません。配信時に元のstampを更新せず、対象TFは直接観測時だけ配信します。

入力・追跡結果の期限は既定で0.3秒です。JSONの`tracking.input_timeout_s`・`tracking.track_timeout_s`やROSパラメータで変更できます。固定対象TFの除外接続、静止時の捕捉、認識精度と期限の妥当性は、実際の地図・bag・実機で確認してください。

## 検証

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/LiDAR-Perception-System
python3 scripts/benchmark_cuda.py /path/to/rosbag_directory --offset 0 --seconds 180 --output /tmp/lps_benchmark.json
```

ベンチマークにはrosbagディレクトリまたは`.db3`ファイルを直接指定できます。`--offset`は記録開始からの秒数です。`--include-traces`を付けると全座標履歴を出力するため、生成ファイルは機密の大会データとして扱ってください。比較対象の`/opponent_robot/bucket_target`は既存アルゴリズムの結果で、正解ラベルではありません。精度の合否には、映像や実測位置に基づく別の正解データが必要です。精度を評価するときは観測不能な区間とTF欠落区間を分けて集計してください。

CPUのsupport_tracker_testはリポジトリに含まれます。鮮度監視の追加試験はローカルのファイルがある場合だけ登録します。GPUノードのビルド成功と、実際の認識精度・GPU動作は別に検証します。


## 新方式：相手座標系での点群蓄積と開口中心推定

`bucket_opening_tracker`は、設定した範囲内の相手クラスタを捕捉し、車体下部の平面ICPから位置・回転を推定します。各フレームの3次元点を相手座標系へ変換し、直近1秒を蓄積して、水平なバケツ開口上端の円・円弧を探します。出力の位置は、支持部重心ではなく**開口面上の円中心**です。

CPUとEigenで動作し、PCL・ESDF・事前PCDは使いません。初期版はbagで検証するための構成で、実機の照準・射出への接続はしていません。セッティング専用の学習操作は不要です。

### 設定と起動

GPUを使わないビルド例です。CUDA方式もビルドする場合は`BUILD_GPU_NODES=OFF`を省いて、対象GPUのarchitectureを指定してください。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_perception_system --cmake-args -DBUILD_GPU_NODES=OFF
source install/setup.bash
cp src/LiDAR-Perception-System/config/bucket_opening.json /tmp/bucket_opening.json
```

`/tmp/bucket_opening.json`の`opponent_roi`を、RViz上の`map`座標で相手を探索するXY矩形へ変更します。配布テンプレートの`null`は意図的な未設定値で、保存済みの有効な設定も指定設定もない場合は、起動前にエラーで停止します。次は書式例であり、実フィールドの設定値ではありません。

```json
"opponent_roi": {"min_x": -2.0, "max_x": 2.0, "min_y": 1.0, "max_y": 5.0}
```

設定済みJSONを`~/.config/lidar_perception_system/bucket_opening.json`へ保存すると、`config_file`引数を省略してもその設定を読みます。`XDG_CONFIG_HOME`が設定されていれば、その配下の`lidar_perception_system/bucket_opening.json`を使います。明示した`config_file`が最優先です。保存設定がなければ配布テンプレートを使い、ROI未設定や不正JSONはノード起動前にエラーとして停止します。起動ログに選択した設定パスを出します。

検出ノードが異常終了した場合は、同じlaunchのguardやRVizも終了させます。空のRVizだけが残って検出しているように見える状態を避けます。自己位置推定を再計算するbag再生では`--clock`を付け、`/livox/lidar /livox/imu`だけを再生し、準備区間からTFとモデルを作ってください。

LiDAR入力と点群時刻のTFがある状態で起動します。

```bash
ros2 launch lidar_perception_system bucket_opening_rviz.launch.py config_file:=/tmp/bucket_opening.json
```

RVizなしでは`bucket_opening.launch.py`を使います。bag途中再生の例は次のとおりです。`bag_path`は固定TFを事前配信するための引数です。動的TFはbagの記録値を使う例なので、自己位置推定による同じTFの再計算とは同時に行いません。

```bash
ros2 launch lidar_perception_system bucket_opening_rviz.launch.py config_file:=/tmp/bucket_opening.json use_sim_time:=true bag_path:=/absolute/path/to/bag
ros2 bag play /absolute/path/to/bag --clock --start-offset 936 --topics /livox/lidar /tf /tf_static
```

上の2コマンドは別ターミナルで実行します。新しい端末ではROSとworkspaceのsetupをsourceしてください。

### 判定と追跡

- 上方の開口形状または底面と側面の組合せを識別できる場合や、探索ROI内の候補が一つの場合は停止中も初期捕捉できます。複数の形だけでは区別できない候補がある場合は、4観測以上・0.2秒以上の対応付けと0.25m超の移動を根拠に選択します。既存のCPU支持部追跡の移動履歴を再利用し、点数の多い固定物だけでモデルを初期化しません。既知の`desk_*`・`bucket_*`・`flag_*`固定TFが配信されていれば、その周辺も初期候補から除きます。
- 捕捉後は予測した車体領域を回収して平面ICPを行います。疎なスキャンで車体が複数クラスタに割れても、毎回のクラスタ重心へ座標原点を置き直しません。規則的な点配置での一周期のずれを避けるため、速度予測と、voxel化した本体重心から作る近傍初期位置の両方をICPで照合します。最適化は外れ値をtrimし、品質判定と初期値の選択には全対応点の残差と対応率を用います。重心をそのまま姿勢にはしません。
- 開口は高さ1.2〜2.1m、半径110〜150mmで探索します。公称外径からの半径136.5mmは仮説であり、開口内径の実測値ではありません。
- 初期条件は異なる12ボクセル以上、円弧120度以上、半径方向残差15mm以下、上面の高さ残差4mm以下です。側面が上に続く断面を除き、ふちの半径方向の広がり3mm以上と、最新フレームの支持点も要求します。最新点でも約2.8度幅の角度区間2つ以上にふちの広がりが必要です。開口中心から半径の65%以内に同じ高さの点が3ボクセル以上あれば、閉じた上面として拒否します。
- 3観測で確定します。蓄積した過去点だけでは観測時刻を更新しません。欠測時は観測できた車体姿勢で学習済みの水平取付位置を変換し、`PREDICTED`として扱います。予測は0.3秒までで、古い結果は別CPUノードの鮮度監視でも無効になります。
- 高さが3cmを超えて変化したら開口付近の蓄積を破棄し、再確定するまで無効にします。車体の蓄積は維持します。昇降中の連続追跡・昇降速度推定は行いません。
- 開口形状も移動履歴も確認できない初期候補は2秒で探索をやり直します。初期捕捉の失敗位置は短時間だけ候補から外します。追跡のリセットでは新しいIDを割り当てます。
- 開口・底面の確定または移動の根拠を得た相手候補は、車体を観測できている間は停止しても本体モデルとIDを保持します。開口結果は0.3秒で無効になり、本体の移動を開口の観測として配信し続けません。本体も2秒間観測できなければリセットします。

JSONの`filter`・`robot`・`accumulation`・`registration`・`opening`・`bottom`・`tracking`で各条件を変更できます。距離はm、時間は秒、円弧角はradです。設定値には有限値・範囲・計算量上限の検査があります。

### バケツ底面の自動認識

相手の取付位置・高さを指定せず、相手座標系の蓄積点群から底面を探索します。ルール指定の共通バケツの外径273mm・高さ255mmと水平設置を形状の事前情報にします。底面半径の許容値は検出用の仮説で、ルールに記載された実測寸法ではありません。画像上で指定された座標や高さを検出コードへ与える処理はありません。

高さごとの点群を連結成分に分け、ほぼ水平でコンパクトな円盤状の面を検出します。面の内部にも支持点を要求し、円弧だけの側面や大きい天板から任意に切り出した円を拒否します。さらに、その上へ続く側面の半径・高さ方向の広がりを確認し、想定したバケツ高さを越えて側面が続く構造や、内部を貫く太い支柱を除外します。複数の同程度の候補があれば確定しません。

最新scanでも底面の支持点を要求し、`tracking.confirm_hits`回の位置・高さの一致で`bottom.valid=true`にします。車体や過去の蓄積だけで底面を有効にし続けません。観測欠落時は無効、入力停止時はROS時計とsteady時計の監視により既定0.3秒で無効にします。時計停止・巻き戻し・不正点群・TF欠落でも古い有効結果を維持しません。高さ変化時は上部モデルを破棄して再確認します。開口中心を底面高さと定数から生成することはありません。

JSONの`bottom`で次を設定できます。

| キー | 既定値 | 内容 |
| --- | --- | --- |
| `bucket_height` / `height_tolerance` | 0.255 / 0.035m | 共通バケツの高さと許容幅 |
| `min_radius` / `max_radius` | 0.075 / 0.135m | 底面円の探索半径 |
| `plane_distance` / `max_plane_rmse` | 0.012 / 0.008m | 平面支持点の距離と残差上限 |
| `max_tilt_rad` | 約0.175rad | 平面法線の傾き上限 |
| `connection_m` | 0.035m | 平面候補の連結距離 |
| `side_tolerance` / `min_side_span` | 0.030 / 0.12m | 側面半径の許容誤差と観測高さ幅 |
| `min_plane_points` | 20 | 蓄積面の最低支持点数 |
| `min_current_points` | 4 | 最新scanの最低面支持点数 |
| `min_side_points` | 6 | 上側側面の最低支持点数 |
| `max_body_inference_s` | 0 | 0は同じ車体を観測できる間、取付点モデルを保持。正値なら面欠測時の補完上限（最大10秒） |

探索する底面高さは、開口の許容高さからバケツ高さを引いて導出します。取付位置の探索範囲は相手ROIと`robot.crop_margin`にも依存します。底面・側面が完全に隠れる、透明材から十分な反射が返らない、同形状の物体と区別できない場合は未確定です。ルールの共通バケツを対象とする方式であり、任意の形状・遮蔽・機体に対する成功保証ではありません。別の実ロボットでの検出率は未受入です。

### 旧マーカーから新しい底面表示へ切り替える

`/moving_bucket_detector/markers`は従来方式の出力です。`moving_bucket.launch.py`や`moving_bucket_rviz.launch.py`で新しい底面検出は起動しません。底面の表示は`bucket_opening_rviz.launch.py`を起動し、`/bucket_opening_tracker/bottom_markers`（認識は橙、面追跡は黄、車体からの補完は水色）と`/bucket_opening_tracker/bottom_cloud`を確認してください。RVizのFixed Frameは`map`です。`/bucket_opening_tracker/markers`は開口の結果です。

既存RVizを使う場合はMarkerArray表示を追加して`/bucket_opening_tracker/bottom_markers`を選びます。別途`bucket_opening.launch.py`で新ノードを起動する必要があります。最初の形状認識前、車体登録失敗、底面観測の期限超過、入力停止時には底面markerを削除します。

比較画像の「横位置」は画像用に原点を移した相対座標で、mapのX/Y座標そのものではありません。画像上の横位置をそのままRVizの座標へ指定しないでください。設定ファイルは再起動後も残る場所へ保存してください。`input_topic`を`/dynamic_cloud`にする場合は、そのpublisherが自己位置推定から配信されていることも確認します。

### 出力とRViz

| 名前 | 内容 |
| --- | --- |
| `/bucket_opening_tracker/bottom` | `BucketBottom`：従来どおり、形状認識が確定した底面だけvalid（DIRECT）。開口の照準点ではない |
| `/bucket_opening_tracker/bottom_track` | `BucketBottom`：認識済み底面の追跡結果。DIRECT／PLANE_TRACKED／BODY_INFERREDと元の面観測stampを区別 |
| `/bucket_opening_tracker/bottom_cloud` | 現在scanの追跡面支持点（map座標）、車体からの補完・無効時は空 |
| `/bucket_opening_tracker/bottom_markers` | 底面円・中心球（認識:橙、面追跡:黄、車体補完:水色）とXYZ軸（赤・緑・青）、無効時はすべて削除 |
| `/bucket_opening_tracker/target_raw` | 監視前の`MovingBucketTrack` |
| `/bucket_opening_tracker/target` | 独立CPU監視後の開口中心・水平速度・不確かさ・valid |
| `/bucket_opening_tracker/selected_cloud` | 選択した相手領域の最新3次元点群（map座標） |
| `/bucket_opening_tracker/accumulated_cloud` | 相手運動を補償した蓄積点群を現在のmap座標へ戻したもの |
| `/bucket_opening_tracker/markers` | 監視後の開口円・中心・速度・状態 |
| `/bucket_opening_tracker/robot_markers` | 相手局所座標の軸 |
| `/bucket_opening_tracker/diagnostics` | 処理件数・候補数・ICPの残差／対応率・蓄積点数・開口仮説の各段階・処理時間 |
| `map → lps_opponent_<id>` | 車体姿勢を観測できた時刻の局所座標TF |
| `map → bucket_bottom` | bottom_trackの底面中心の動的TF。新しい車体姿勢に同期して認識／追跡が有効なとき配信 |

`DIRECT`は最新点にも裏付けられた開口推定、`PREDICTED`は新しい開口観測のない短期予測です。支持部からの固定高さ出力はしません。新方式の`support_points`は最新フレームで開口円を支持した点数です。開口の緑／青マーカーは監視後の結果から作り、無効時は削除します。点群と座標軸は検出過程の可視化で、有効な照準結果とは区別してください。

`BucketBottom`型に`observation_mode`を追加しています。この型を使う独自のC++ノードがある場合は再ビルドし、購読ノードも再起動してください。

底面を一度形状認識した後は、相手の局所座標に取付位置を保存します。毎scanで円盤全体を再認識する必要はありません。現在の底面近傍に最低4点の二次元的な広がりを持つ平面があれば、高さ・法線を補正して`PLANE_TRACKED`として追跡します。局所XYと半径は最初に認識した値を引き継ぐため、部分面だけから新しいバケツを認識した意味ではありません。面が隠れた区間は現在の車体ICP姿勢から`BODY_INFERRED`として位置を更新します。既定では同じ車体を観測できる間、学習した取付点を車体モデルの一部として保持します。`bottom.max_body_inference_s`を正値にすると、その秒数を面の最終観測から超えた時点で停止し、全体形状の再認識が必要です。車体登録の失敗・別track・観測との矛盾があるフレームでは追跡出力を無効にします。単発の別高さ候補で学習済みモデルを破棄せず、新しい底面が継続して形状認識された時点で取付位置を置き換えます。完全遮蔽中の昇降や取付位置変更は観測できず、BODY_INFERREDの底面位置には追従できません。このモードは最後に認識した取付位置が車体に対して変わらないというモデル予測です。

底面TF `bucket_bottom`は`/bucket_opening_tracker/bottom_track`の有効結果から自動配信します。原点は底面中心、Z軸は底面法線、X軸は相手局所Xを底面へ投影した方向、Y軸は右手系です。円形底面からyawを測定した意味ではありません。親は`target_frame`（既定`map`）、TFのstampは現在の追跡時刻です。`last_observation_stamp`は面を観測した時刻を保持し、車体だけの更新では延長しません。`BODY_INFERRED`のconfidenceとcurrent_pointsは0です。TFの過去履歴は受信bufferに残るため、利用側はbottom_trackのvalid・observation_mode・観測時刻も確認してください。入力やROS時計の停止時には、steady時計でも既定0.3秒で失効します。

開口TFは既定で無効です。JSONの`publish_tf`を有効にすると、新方式のguardは`map → bucket_opening`を直接観測時だけ配信します。この設定は底面TFには影響しません。旧方式の`moving_bucket`とは別名です。guardの`child_frame`パラメータの既定値は従来どおり`moving_bucket`です。

### bagでの比較

```bash
ros2 run lidar_perception_system benchmark_opening.py /absolute/path/to/bag --config /tmp/bucket_opening.json --offset 936 --seconds 60 --output /tmp/opening_report.json
```

旧CUDA方式と同じ入力で比較する場合は、`--baseline-executable /absolute/path/to/moving_bucket_detector`を追加します。必要なら`--baseline-config`も指定します。CPUのみのinstallには旧GPU実行ファイルがないため、比較には別途ビルド済みの実行ファイルが必要です。

自己位置推定を再実行する場合は、`gn10_pointcloud_localization`をビルド・sourceした環境で`--recompute-localization`を付けます。スクリプトが`localization.launch.py use_sim_time:=true`を起動し、bagの`/livox/lidar /livox/imu`だけを再生します。記録済みTFは流しません。このモードではJSONの`input_topic`を`/dynamic_cloud`へ変更して、静的地図との差分点群から相手の局所モデルを作る比較もできます。ここでの差分は地図との幾何的な不一致であり、停止中の相手も含みます。自己位置推定の誤差や地図上の固定物との近接による点の除去には注意が必要です。

試合前からモデルを蓄積するには`--warmup-seconds`を指定します。例えば`--offset 936 --warmup-seconds 36 --seconds 180 --recompute-localization`は900秒から再生し、936秒から180秒間の計測だけを件数に集計します。準備区間と試合区間で追跡状態をリセットしません。診断のカウンタ・ピークは準備区間も含む生存期間のスナップショットなので、試合中の件数とは区別してください。記録に存在しないセッティング区間の点群は復元できません。

比較レポートの`bottom`は直接認識だけ、`bottom_tracking`はDIRECT／PLANE_TRACKED／BODY_INFERREDの件数を分けて記録します。件数や有効出力間の最大間隔は真値付き精度・連続認識率ではありません。

終了時刻はbagのROS clockで判定し、開始処理の待ち時間によって集計区間が短くならないようにします。`evaluation_completed`は指定した区間の終端まで再生できたかを表します。記録の途中終了やclockの停止で区間を完了できなければスクリプトは異常終了します。

スクリプトは専用のlocalhost ROS domainで再生し、自分が起動したプロセスだけを終了します。JSONへ処理時間、直接観測・予測・無効の件数、監視後に通過した計測数、無効理由を保存します。guardによる同じ計測時刻の再配信は有効計測数へ重複計上しません。`opening_confirmed`は、集計区間で判定条件を満たす開口候補が1件以上出たかを表します。スクリプトの正常終了やこのフラグは、開口の正しさや精度の合格ではありません。`bottom.valid_measurements`と`bottom.track_ids`は専用底面トピックの集計で、開口の結果と区別します。どちらも真値付き精度を意味しません。座標履歴は出力しません。

### 制約と検証範囲

フレーム間の平面運動を補償する初期版です。スキャン内部の移動、上下動・大きな傾き、疎な／対称な形状による姿勢の曖昧さは残ります。1方向からの蓄積で未観測の裏側は復元できません。薄い・透明なふちで十分な上端点が得られない場合も、車体重心で代替せず無効になります。

移動の根拠だけでは、相手ロボットと人などの動く物体を意味的に区別できません。自己位置の誤差や遮蔽による重心変化も影響するため、探索ROIと実データでの対象の照合が必要です。診断の`robot_has_motion_evidence`は本体候補の移動履歴を示し、開口の有効性を表しません。

合成点群では開口中心、部分円弧、並進・回転、静止捕捉、欠測、昇降後の再捕捉、候補の取り違え、時刻異常を検証します。追加の合成／ROS結合試験はローカルファイルがある場合だけCMakeへ登録し、既存方針に従ってGit管理から除外します。実データの絶対精度や実機での認識率は未受入です。`confidence`は校正された確率ではありません。

手元の実bagでは、記録TFを使う936秒から180秒間の検証で開口確定0件でした。自己位置を再計算し、試合前36秒の蓄積と地図差分点群を使った比較ではロボット上面を開口と誤認したため、中央の占有点と最新のふち形状を検査するよう修正しました。修正後の180秒間でも有効な開口出力は0件で、実データでの開口認識は未達です。処理エラー・キュー破棄は0件でしたが、開口点の可観測性と検出条件を引き続き検証する必要があります。ビルド・合成試験の成功を実bagでの認識成功とは扱いません。

その後、底面の対応箇所を照合したところ、候補選択が別のクラスタへ偏っていたことも判明しました。候補が複数ある場合は開口形状または継続した移動履歴を選択根拠に変更し、実ROS再生の同時刻で指定底面付近の点が選択・蓄積点群に含まれることを確認しました。修正後の準備36秒＋評価35秒では349件の開口結果はすべて無効でした。底面を含む領域の抽出と、開口中心の正しい連続追跡は別の到達点で、後者は未達です。

底面自動認識の追加後、取付高さ4種類・向き3種類・底面半径3種類の合成点群、部分遮蔽・小さい傾き・昇降・誤候補・最新面欠測を検証しています。実bagの16分04秒付近では、指定位置を検出条件に与えず底面候補が専用トピックで確定しました。単一の実ロボットでの対応確認であり、他機体すべての精度を保証するものではありません。

底面実装直後の実ROS再生では、準備36秒と936秒から180秒の区間を完了し、底面の確定計測5件（2つの本体ID）、開口の有効計測0件でした。16分04秒付近はユーザー指定の底面との対応を確認しましたが、全出力の正解ラベルや連続認識率は未評価です。支持点不足では未確定になるため、連続認識の実データ受入は未達です。

異なる3種類の車体を並進・回転させる追加試験で、周期的な点配置がICPの位置ずれを起こす問題を検出し、全対応点での残差評価と複数初期位置による照合で修正しました。修正後の通常installで同じ準備36秒＋評価180秒を再生すると、入力1,794件に対して底面の確定出力26計測（本体ID2）、開口の有効出力0件でした。指定箇所では高さ約1.759mの底面との対応を確認しました。26出力すべての正誤や連続認識率は未評価で、点不足区間を含めた連続認識は未達です。
