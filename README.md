各ファイルの説明

waiacc.py 　　　ジャイロセンサから加速度、角速度を出力
Madgwick2.py　　加速度、角速度データから四元数を出力
quaternion.py  四元数の計算。Madgwick2.pyで使用
sensor.py      
  上記３つのフォルダを用いて、一定時間ごとに四元数を出力
  測定時間の間隔を変更するときは、
  waiacc.pyの4行目,sensor.pyの11,21行目の値を変更すること
PID.py         
  PID制御。
  目標値とどれだけずれているかを入力したとき、どれだけ設定値を変えればいいかを出力する
  一般的なものでドローン用にするにはもうひと工夫必要

回路図は
https://www.sunhayato.co.jp/material2/index.php/item?cell003=%E6%95%99%E8%82%B2%E5%AE%9F%E7%BF%92%E3%83%BB%E9%9B%BB%E5%AD%90%E5%B7%A5%E4%BD%9C%E8%A3%BD%E5%93%81&cell004=IC%E3%83%A2%E3%82%B8%E3%83%A5%E3%83%BC%E3%83%AB&name=%E5%8A%A0%E9%80%9F%E5%BA%A6%E3%83%BB%E3%82%B8%E3%83%A3%E3%82%A4%E3%83%AD%E3%82%BB%E3%83%B3%E3%82%B5%E3%83%BC%EF%BC%88%E3%83%AC%E3%83%99%E3%83%AB%E5%A4%89%E6%8F%9BIC%E4%BB%98%E3%81%8D%EF%BC%89%E3%83%A2%E3%82%B8%E3%83%A5%E3%83%BC%E3%83%AB%E3%80%80MM-TXS04&id=763&label=1
のI2C接続参照
         
