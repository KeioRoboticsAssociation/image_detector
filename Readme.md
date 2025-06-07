# image_detector

ROS 2パッケージで、カメラ画像から直線とカラーボールを検出します。

## 概要

このパッケージは、カメラから取得した画像に対して以下の処理を行います：
- **直線検出**: Hough変換を使用して画像内の直線を検出
- **カラーボール検出**: HSV色空間でのしきい値処理により、指定された色のボールを検出

## 機能

### 直線検出
- Cannyエッジ検出とHoughLinesP変換を使用
- 検出された直線の始点と終点の座標を出力

### ボール検出
- HSV色空間での色フィルタリング
- 輪郭検出と重心計算により、ボールの位置を特定
- 複数の色のボールを同時に検出可能（設定ファイルで定義）

## 依存関係

- ROS 2 (Humble以降推奨)
- OpenCV
- 以下のROS 2パッケージ:
  - rclcpp
  - sensor_msgs
  - geometry_msgs
  - image_transport
  - cv_bridge

## インストール

```bash
# ワークスペースのsrcディレクトリにクローン
cd ~/ros2_ws/src
git clone [このパッケージのURL]

# 依存関係のインストール
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# ビルド
colcon build --packages-select image_detector
```

## 使用方法

### 1. 設定ファイルの編集

`config/config.yaml`を編集して、検出パラメータを調整します：

```yaml
camera:
  input_topic: "/camera/resize_hsv/image_raw"  # 入力画像トピック

detection:
  line:
    rho: 1                # Hough変換のρパラメータ
    theta: 1.0            # Hough変換のθパラメータ
    threshold: 50         # 直線検出のしきい値
    min_line_length: 50   # 最小直線長
    max_line_gap: 10      # 直線の最大ギャップ

  balls:
    - name: "red"         # 赤いボール
      hsv_lower: [0, 120, 70]
      hsv_upper: [10, 255, 255]
    - name: "blue"        # 青いボール
      hsv_lower: [100, 150, 0]
      hsv_upper: [140, 255, 255]
```

### 2. ノードの起動

```bash
# ソースの読み込み
source ~/ros2_ws/install/setup.bash

# パラメータファイルを指定して起動
ros2 run image_detector detector_node --ros-args --params-file src/image_detector/config/config.yaml
```

## トピック

### Subscribe
- **入力画像**: `/camera/resize_hsv/image_raw` (sensor_msgs/Image)
  - BGR8フォーマットの画像を受信

### Publish
- **検出された直線**: `/detection/lines` (image_detector/LineSegmentArray)
  - 各直線の始点と終点の座標を含む配列
- **検出されたボール**: `/detection/balls` (image_detector/BallPositionArray)
  - 各ボールの色と中心座標を含む配列

## カスタムメッセージ

### LineSegment.msg
```
geometry_msgs/Point start  # 直線の始点
geometry_msgs/Point end    # 直線の終点
```

### BallPosition.msg
```
string color               # ボールの色
geometry_msgs/Point position  # ボールの中心座標
```

### LineSegmentArray.msg
```
image_detector/LineSegment[] lines  # 直線の配列
```

### BallPositionArray.msg
```
image_detector/BallPosition[] balls  # ボールの配列
```

## パラメータ調整のヒント

### 直線検出の改善
- `threshold`: 値を下げると、より多くの直線を検出（ノイズも増える）
- `min_line_length`: 短い直線も検出したい場合は値を下げる
- `max_line_gap`: 途切れた直線をつなげたい場合は値を上げる

### ボール検出の改善
- HSV範囲の調整：
  - H (Hue): 0-180の範囲で色相を指定
  - S (Saturation): 0-255の範囲で彩度を指定
  - V (Value): 0-255の範囲で明度を指定
- 照明条件に応じてHSV範囲を調整することが重要

## トラブルシューティング

### 画像が受信されない場合
```bash
# トピックの確認
ros2 topic list
ros2 topic echo /camera/resize_hsv/image_raw
```

### 検出結果の確認
```bash
# 直線検出結果
ros2 topic echo /detection/lines

# ボール検出結果
ros2 topic echo /detection/balls
```

## 今後の改善点

- [ ] 動的パラメータ再設定の実装
- [ ] 検出結果の可視化ノードの追加
- [ ] より多くの色設定のサポート
- [ ] 検出精度の向上（ノイズ除去フィルタなど）

## ライセンス

TODO: ライセンスを記載

## 作者

メンテナー: root <ymrs1122@gmail.com>