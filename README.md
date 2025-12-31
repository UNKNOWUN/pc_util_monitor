# pc_util_monitor

PCのCPU使用率(%)およびGPU使用率(%)を取得し、ROS 2のトピックとして配信するパッケージです。

## 使用方法

以下の手順に沿って行ってください。

- リポジトリをクローンします。

  ```bash
  git clone https://github.com/UNKNOWUN/pc_util_monitor.git
  ```

- ディレクトリに移動します。

  ```bash
  cd pc_util_monitor
  ```

- テストを実行します（ワークスペースを自動生成し、build / test を行います）。

  ```bash
  chmod +x test/test.bash
  ./test/test.bash
  ```

- 実行例（ROS 2 ワークスペースに配置してノードを実行します）。

  ```bash
  mkdir -p ~/ros2_ws/src
  cp -r ./ ~/ros2_ws/src/pc_util_monitor
  cd ~/ros2_ws
  colcon build --symlink-install --packages-select pc_util_monitor
  source install/setup.bash
  ros2 run pc_util_monitor pc_util_monitor
  ```

- 別ターミナルでトピックを確認します。

  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 topic echo /pc/cpu_util
  ros2 topic echo /pc/gpu_util
  ros2 topic echo /pc/monitor_status
  ```

### 必要なソフトウェア
- ROS 2 Humble
- Python 3
- `colcon` / `ament`（ROS 2 の標準ビルド・テスト環境）
- （任意）`nvidia-smi`（GPU使用率を取得したい場合）

### テスト環境
- Ubuntu 22.04
- ROS 2 Humble

## ライセンス

- このソフトウェアパッケージは、3条項BSDライセンスの下で、再配布および使用が許可されています。
- このパッケージのコードの一部は、以下の講義資料  
  （CC-BY-SA 4.0 by Ryuichi Ueda）を参考にして作成されています。

  - https://github.com/ryuichiueda/slides_marp/tree/master/robosys2025

- © 2025 Toshiaki Kou

