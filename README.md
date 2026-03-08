<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Image to Position

## 概要

画像認識によって検出された結果(BoundingBox, Keypoints, Masks)を元に、点群を重ね合わせることで3次元座標(TF)化するパッケージです。

RGB-Dカメラの点群(sensor_msgs/PointCloud2)と画像認識結果を用いて、物体やキーポイントの3次元座標を推定します。
本パッケージはROS 2の**Lifecycle Node**として実装されており、標準的なROS 2のライフサイクル管理コマンドで制御可能です。

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## セットアップ

ここで，本リポジトリのセットアップ方法について説明します．

### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS | Jazzy Jalisco |
| Python | 3.12 |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)を参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. ROSの`src`フォルダに移動します．
   ```sh
   $ cd ~/colcon_ws/src
   ```
2. 本レポジトリをcloneします．
   ```sh
   $ git clone -b jazzy-devel https://github.com/TeamSOBITS/image_to_position.git
   ```
3. 必要な依存パッケージをインストールします．
   ```sh
   cd image_to_position/
   bash install.sh
   ```
4. パッケージをコンパイルします．
   ```sh
   colcon build --symlink-install
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## 実行・操作方法
本パッケージはROS 2のLifecycle Nodeとして動作します。起動後、`unconfigured`状態から`active`状態に遷移させることで処理が開始されます。

### ノードの起動とライフサイクル制御
例: `bbox_to_3d`ノードの起動
```sh
# ノードの起動 (Active状態で起動しない場合)
ros2 run image_to_position bbox_to_3d

# ライフサイクル状態の確認
ros2 lifecycle get /bbox_to_3d

# 設定 (Configure)
ros2 lifecycle set /bbox_to_3d configure

# 起動 (Activate)
ros2 lifecycle set /bbox_to_3d activate
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### パラメータ設定 (Spatial Clipping)
点群処理の最適化のため、PCLの設定を行います。[config](./config/)フォルダにあるyamlファイルを編集してください。


- `base_frame_name`:
  base_frame_nameは3次\元化する上で何を基準フレームとして座標を得るかです．\
  ロボットの座標を基準にする場合，base_footprintとなる．\
  ロボットの足元の中心を原点(0,0,0)として各認識した物体の3次元座標を生成します．

- `bbox_topic_name`:\
  bbox_topic_nameはBoundingBoxのトピック名です．
  具体的には，sobits_interfaces/BoundingBoxes型のメッセージが飛んでいるトピック名を指定する．\
  これはSOBITSが独自に作ったカスタムROSメッセージであるためsobits_interfacesをgit cloneしてある必用があります．\
  しかし，このパッケージに依存しているパッケージのinstall.shで既にgit cloneされているはずです．

- `cloud_topic_name`:\
  cloud_topic_nameは点群のトピック名です．
  具体的には，sensor_msgs/PointCloud2型のメッセージが飛んでいるトピック名を指定する．\
  BoundingBoxの情報から，その領域に点群を飛ばします．
  そのとき，物体の距離感とともに点群のクラス分けを行い位置を取得します．

- `img_topic_name`:\
  img_topic_nameはこの画像認識をしている画像のトピック名です．\
  具体的には，sensor_msgs/Image型のメッセージが飛んでいるトピック名を指定する．\
  画像と点群の関係を見るために参照している．

- `x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max`:\
  光学フレームにおける有効な空間範囲をメートル単位で指定します。これにより、不要な背景を除去し処理を高速化します。

- `cluster_tolerance`:\
  どの程度離れた点群までは同一の物体とみなすかのしきい値です．\
  BoundingBox内に点群を飛ばした場合に，対象物に点群があたり，しきい値いないにある点群を1物体とみなしクラス分けを行います．
  そのため，あまり大きくすると点群1つ1つの探索範囲が広がり処理が遅くなってしまいます．

- `min_clusterSize`:\
  どの程度の数以下の点群の集まりは対象物の点群から棄却するかのしきい値です．\
  点群をクラス分けした際に，この数以下の点群数だったらノイズとみなし棄却します．

- `max_clusterSize`:\
  どの程度の数以上の点群の集まりは対象物の点群から棄却するかのしきい値です．\
  点群をクラス分けした際に，この数以上の点群数だったら全く別の対象物(物体だったら床の点群など)を捉えてしまったとみなし棄却します．

- `noise_point_cloud_range`:\
  対象の物体の点群からノイズ面を除去し，中心座標に近づけるため除去量です．\
  クラス分けした点群から物体を抽出した後，床や背後の壁，左右の壁などx,y,z方向に点群をこの値分，更にカットします．
  こうすることで，より物体の部分のみにかかる点群に絞ることができます．
  しかし値を大きくしすぎると，物体分の点群まで多く削いでしまうため注意が必用です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## 参考文献

* [ROS Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [Point Cloud Library (PCL)](https://pointclouds.org/)
* [Perception PCL](https://github.com/ros-perception/perception_pcl)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/image_to_position.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/image_to_position/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/image_to_position.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/image_to_position/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/image_to_position.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/image_to_position/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/image_to_position.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/image_to_position/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/image_to_position.svg?style=for-the-badge
[license-url]: LICENSE
