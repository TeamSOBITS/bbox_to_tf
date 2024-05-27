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

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Services
 * /speech_recognition (Service: speech_recognition_whisper/SpeechRecognitionWhisper)

 <p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

現時点のバッグや新規機能の依頼を確認するために[Issueページ](issues-url) をご覧ください．


<!-- 参考文献 -->
## 参考文献

* [ROS Noetic](http://wiki.ros.org/noetic)

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/speech_recognition_whisper.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/speech_recognition_whisper/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/speech_recognition_whisper.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/speech_recognition_whisper/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/speech_recognition_whisper.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/speech_recognition_whisper/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/speech_recognition_whisper.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/speech_recognition_whisper/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/speech_recognition_whisper.svg?style=for-the-badge
[license-url]: LICENSE

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[]: 
