# 2輪型倒立振子
ESP32内の変数をアプリ経由で変更可能なblynkを用いた倒立振子  
タミヤと秋月電子のパーツで作成可能  
<p style="text-align: center"><img src="./img/2w_pendulum.gif" style="width: 400px;"/></p>

# 構成
内蔵した6軸センサとセンサレスモータ制御によるPID倒立振子です。  
以下の2つでPID制御しています。
```
姿勢角:     角度と角速度のPD制御  
モータ角度:  速度と距離(速度の積分値)のPI制御  
```
部品構成は以下の通りです。

|      部品     |  型番  |
| :----------  | :---- |
| MCU          | [ATOM Matrix](https://akizukidenshi.com/catalog/g/gM-17215/)(中身はESP32)  |
| Motor        | [ダブルギアボックス(左右独立4速タイプ)](https://www.tamiya.com/japan/products/70168/index.html)<br>(ギア比114.7:1にして使用)  |
| Body         | [ユニバーサルプレートセット](https://www.tamiya.com/japan/products/70098/index.html)  |
| Motor Driver | [DRV8835](https://akizukidenshi.com/catalog/g/gK-09848/)  |
| Battery Box  | [単3x4](https://akizukidenshi.com/catalog/g/gP-02671/)  |


# セットアップ
## ESP32の環境構築
  1. Arduino IDEを開く
  2. file -> Preferences...を開く
  3. 追加ボードマネージャに "https://dl.espressif.com/dl/package_esp32_index.json" を追加
  4. tool -> board(board manager)を開く
  5. "esp32"とでたものをインストール

  ```
  ESP32は内部でesptool.pyが動きます。  
  M1Macを使用している場合にはPython周りで注意!
  ```

## 必要ライブラリのインストール
以下のライブラリをインストール
```
blynk-library
FastLED
```
blynkライブラリのインストールは[こちら](https://github.com/blynkkk/blynk-library)

## blynk(アプリ)のインストール
"blynk"と打ちアプリ検索  
iOSであれば[こちら](https://itunes.apple.com/us/app/blynk-control-arduino-raspberry/id808760481?ls=1&mt=8)から入手  
```サービス終了している可能性がある```

