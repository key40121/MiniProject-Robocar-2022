# インストール手順

Ubuntu 20.04 LTSにて確認
Raspberry Pi(Raspbian)では構築できませんでした。おそらくARMアーキテクチャ上ではライブラリが起動しないためだと思います。

## 1. ライブラリのインストール
必要なライブラリ

| name | description |
| --- | --- |
| urg_library | 測域センサ(2D LiDAR)|
| opencv2 | openCV(画像処理) |
| rc110_v2 | Robocar操作 |

1. robocar環境をセットアップするファイルのコピー
まず初めにホームディレクトリにファイル一式を移動します。


2. rc110_v2のインストール
/lib/rc110_v2を作業を行うPCのlibに移動します
```
$ sudo mkdir /usr/local/lib/zmp
$ sudo cp -r ~/robocar/lib/rc110_v2 /usr/local/lib/zmp/
```
を実行します。

3. urg_libraryのインストール
urg関係のライブラリをインストールします。
```
$ cd ~/robocar/setup
$ unzip urg_library-1.2.0.zip
$ cd urg_library-1.2.0
$ make
$ sudo make install
```

4. OpenCVのインストール

時間ができたらやります。

5. includeファイルの移動
Robocarをコンパイルする際に必要なステアリングやセンサを一元的に管理するincludeファイルを移動します。
```
$ sudo mkdir /usr/local/include/zmp
$ sudo cp -r ~/robocar/setup/include/rc110_v2 /usr/local/include/zmp/
```

## 2. ライブラリを認識させる。
ライブラリをインストールしたり移動するダメではLinux側はコンパイル時に読み込もうとしてくれないのでライブラリを認識させます。
```
$ sudo vi /etc/ld.so.conf
```
を実行すると中身が
```
include /etc/ld.so.conf.d/*.conf
```
とのみ記載されています。ここに、
```
include /usr/local/lib/
```
を下に追加し保存。その後
```
$ sudo ldconfig
```
を実行します。

## 3. 環境変数の定義
ライブラリを認識しただけだとLinuxはライブラリの存在は把握しますがどこにあるのかは理解してくれないので、環境変数を定義します。

```
$ nano ~/.bashrc
```
を実行し、最終行に
```
LD_LIBRARY_PATH=/usr/local/lib/:/usr/local/lib/zmp/rc110_v2/:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH
```
を追加します。

