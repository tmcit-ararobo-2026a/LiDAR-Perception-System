# Track鮮度監視（T11）の検証

実施日: 2026-10-14（Asia/Tokyo）。作業ディレクトリ: `/home/lambda/ros2_ws/src`。
対象: 機能別状態型・独立CPU監視、GPU計算ノードのraw出力分離、launch・manifest・模擬試験。
ブランチ: feat/2-track-freshness。
基準HEAD: f7fab24ca02640be51a8ad7409ead50223e7274c。

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/stm32-ros2/log-cpu-guards-final build \
  --base-paths gn10-pointcloud-localization LiDAR-Perception-System \
  --build-base /tmp/stm32-ros2/build-cpu-guards \
  --install-base /tmp/stm32-ros2/install-cpu-guards \
  --packages-select gn10_pointcloud_localization lidar_perception_system \
  --cmake-args -DBUILD_GPU_NODES=OFF
source /tmp/stm32-ros2/install-cpu-guards/setup.bash
ctest --test-dir /tmp/stm32-ros2/build-cpu-guards/lidar_perception_system --output-on-failure
```

- CPUノードのHumble・GCC11ビルド: 成功／終了コード0。GPUなしで監視を起動できる。
- 実DDS故障注入: 成功／終了コード0。入力断・計算結果断・重複再送・期限超過を検証。localhost専用domain、sandbox外で実施。
- LPSの既存support_tracker_testも成功。TFは最終guardのみが所有し、重複した同じstampを再送しない。
- CUDAノードのコンパイル: 成功／終了コード0。`build-gpu`、SM87指定、CUDA13.4。GPU計算の実行はしていない。
- manifestのROS公式schema検査、launchのPython構文: 成功／終了コード0。
- 詳細ログ: `/tmp/stm32-ros2/lps-t11-final-tests.log`。GPUビルドは `gpu-status-build.log`、localizationの未評価指標初期化後は `localization-t05-metrics-build.log`。

## 失敗・未検証

T01初回GPUビルドはMPI wrapperのsocket初期化がsandboxで拒否され、MPI header検出に失敗／colcon終了コード1。
クリーンなbuild-gpuからsandbox外で再実行し、ビルド成功。既存CUBの非推奨API警告、PCLのpcap/png無効警告は残る。
GPU driverはnvidia-smiで通信失敗。正常bag・精度・再探索・静止捕捉・TF除外接続・実機・HILは未実施。
Ready品質・対象設定・校正・許容停止時間の契約は未確定。模擬監視の成功を実機受入と扱わない。
