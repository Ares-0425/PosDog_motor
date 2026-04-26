# PosDog_motor
四脚歩行ロボット「PosDog」のモータードライバプログラムと回路CADデータのレポジトリ
## 動作環境
プログラムはVSCode＋PlatformIO環境で作成しています。
回路CADはKiCADを使用し、JLCPCBでの発注を想定しています。
## ファイル
### プログラム
レポジトリ全体がPlatformIOのプログラムフォルダになっていますが、いじるべきプログラムは"src"フォルダ内に入っています。
- main.cpp  
  メインプログラムファイル。主にSimpleFOC（モーター制御ライブラリ）の設定、SimpleFOC制御ループ、CAN通信で取得した文字列の識別と文字列に応じた動作の実行、キャリブレーション動作の定義、を行っている。
- can.hpp  
  can.cppのヘッダー
- can.cpp  
  can通信用ファイル。設定と受信時のID及び文字列格納。
- dob.hpp  
  dob.cppのヘッダー
- dob.cpp  
  外乱オブザーバーと、外乱オブザーバーを考慮したトルク制御と角度制御の指令値出力。
- identification.hpp  
  identification.cppのヘッダー
- identification.cpp  
  システム同定を一瞬やろうとした時の残骸
### 回路
"drv8311"フォルダ内に入っています。
- drv8311.kicad_pro  
  kicadプロジェクトファイル
- drv8311.kicad_sch  
  kicad回路図ファイル
- drv8311.kicad_pcb  
  kicadPCBファイル
- production
  - drv8311.zip  
    JLCPCB発注用ファイル
## 基本の使いかた
プログラムを書き込む脚に合わせて、main.cppの"#define LEG"の数字を変更する。