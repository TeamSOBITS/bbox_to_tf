<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
<!-- [![MIT License][license-shield]][license-url] -->

# bbox_to_tf

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#launchファイルの構成">launchファイルの構成</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
  </ol>
</details>


<!-- レポジトリの概要 -->
## 概要

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->

画像認識によって検出をしたBoundingBoxを元に点群を重ね合わせることで3次元座標(TF)化するパッケージです．\
基本的にRGB-Dカメラによって検出したBoundingBox(sobits_msgs/BoundingBoxes)と，その画像(sensor_msgs/Image)，そのカメラの点群(sensor_msgs/PointCloud2)を用いて3次元化します．

> [!NOTE]
> このパッケージは基本的に画像処理・物体認識をしたあとに使うものなので，そのパッケージのinstall.shによってインストールされます．
> 例えばYOLOによって検出した物体を3次元座標にするため，YOLOのパッケージのinstall.shによってインストールされます．


<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## セットアップ

ここで，本リポジトリのセットアップ方法について説明します．\
これを依存しているパッケージのinstall.shに書き込んでください．

### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.8 |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)を参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### インストール方法

1. ROSの`src`フォルダに移動します．
   ```sh
   $ cd ~/catkin_ws/src
   ```
2. 本レポジトリをcloneします．
   ```sh
   $ git clone https://github.com/TeamSOBITS/bbox_to_tf.git
   ```
3. パッケージをコンパイルします．
   ```sh
   $ cd ~/catkin_ws/
   $ catkin_make
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## launchファイルの構成

このパッケージは基本的に単体では起動せず，[bbox_to_tf.launch](/launch/bbox_to_tf.launch)を必用な画像処理パッケージのlaunchからincludeして使用します．\
そのため，ここでは[bbox_to_tf.launch](/launch/bbox_to_tf.launch)の中身や構成に関して説明します．

- node_name
  node_nameは，3次元化するノードの名前です．\
  3次元化したい画像認識ノードが1つとは限らないので，複数でもノード名が競合しないようにノード名を指定できるようになっています．
  例えばyolov8とssdによる画像認識からそれぞれTF化したいとき，yolov8_to_tf_nodeとssd_to_tf_nodeのように競合しないようにすることができます．\

- base_frame_name
  base_frame_nameは3次元化する上で何を基準フレームとして座標を得るかです．\
  ロボットの座標を基準にする場合，base_footprintとなる．\
  ロボットの足元の中心を原点(0,0,0)として各認識した物体の3次元座標を生成します．

- bbox_topic_name
  bbox_topic_nameはBoundingBoxのトピック名です．
  具体的には，sobits_msgs/BoundingBoxes型のメッセージが飛んでいるトピック名を指定する．\
  これはSOBITSが独自に作ったカスタムROSメッセージであるためsobits_msgsをgit cloneしてある必用があります．\
  しかし，このパッケージに依存しているパッケージのinstall.shで既にgit cloneされているはずです．

- cloud_topic_name
  cloud_topic_nameは点群のトピック名です．
  具体的には，sensor_msgs/PointCloud2型のメッセージが飛んでいるトピック名を指定する．\
  BoundingBoxの情報から，その領域に点群を飛ばします．
  そのとき，物体の距離感とともに点群のクラス分けを行い位置を取得します．

- img_topic_name
  img_topic_nameはこの画像認識をしている画像のトピック名です．\
  具体的には，sensor_msgs/Image型のメッセージが飛んでいるトピック名を指定する．\
  画像と点群の関係を見るために参照している．

- execute_default
  execute_defaultはデフォルトでTFを出すかどうかです．\
  OFF(False)にしていても，使うときにrun_ctrでON(True)にすることもできる．

- cluster_tolerance
  どの程度離れた点群までは同一の物体とみなすかのしきい値です．\
  BoundingBox内に点群を飛ばした場合に，対象物に点群があたり，しきい値いないにある点群を1物体とみなしクラス分けを行います．
  そのため，あまり大きくすると点群1つ1つの探索範囲が広がり処理が遅くなってしまいます．

- min_clusterSize
  どの程度の数以下の点群の集まりは対象物の点群から棄却するかのしきい値です．\
  点群をクラス分けした際に，この数以下の点群数だったらノイズとみなし棄却します．

- max_lusterSize
  どの程度の数以上の点群の集まりは対象物の点群から棄却するかのしきい値です．\
  点群をクラス分けした際に，この数以上の点群数だったら全く別の対象物(物体だったら床の点群など)を捉えてしまったとみなし棄却します．

- noise_point_cloud_range
  対象の物体の点群からノイズ面を除去し，中心座標に近づけるため除去量です．\
  クラス分けした点群から物体を抽出した後，床や背後の壁，左右の壁などx,y,z方向に点群をこの値分，更にカットします．
  こうすることで，より物体の部分のみにかかる点群に絞ることができます．
  しかし値を大きくしすぎると，物体分の点群まで多く削いでしまうため注意が必用です．


> [!NOTE]
> 基本的に[bbox_to_tf.launch](/launch/bbox_to_tf.launch)を修正するのではなく，このパッケージに依存しているパッケージのlaunchファイルにincludeで呼び出すようにして，競合をさけてください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Services
run_ctrとしてTF化するかしないかのON/OFF(True/False)を切り替えることができます．
基本的に，競合しないようにnode_nameに依存した命名となる．
 * node_name + "/run_ctr" (Service: sobits_msgs/RunCtrl)

### Topic
TF化する他に，base_frame_nameから見た座標や各物体にかかる点群をPublishしている．
基本的に，競合しないようにnode_nameに依存した命名となる．
- 座標
 * node_name + "/object_poses" (Topic: sobits_msgs::ObjectPoseArray)

- 点群
 * node_name + "/object_poses" (Topic: pcl/PointCloud< pcl/PointXYZ >)

 <p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

現時点のバッグや新規機能の依頼を確認するために[Issueページ](issues-url) をご覧ください．


<!-- 参考文献 -->
## 参考文献

* [ROS Noetic](http://wiki.ros.org/noetic)

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/bbox_to_tf.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/bbox_to_tf/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/bbox_to_tf.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/bbox_to_tf/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/bbox_to_tf.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/bbox_to_tf/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/bbox_to_tf.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/bbox_to_tf/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/bbox_to_tf.svg?style=for-the-badge
[license-url]: LICENSE

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[]: 
