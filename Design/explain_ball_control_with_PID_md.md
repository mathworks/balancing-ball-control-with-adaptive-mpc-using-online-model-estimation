
# <span style="color:rgb(213,80,0)">PID制御器によるボールの位置制御</span>
# 初期化
```matlab
system_model_name = 'BallAndPlate_system';
controller_model_name = 'BallAndPlate_PID_Controller';
SDI_view_path = which('circle_move_XY_view.mldatx');

set_slddVal('BallAndPlate_system_data.sldd', 'DELAYMODE', 'ENUM_DELAYMODE.OFF');
set_slddVal('BallAndPlate_system_data.sldd', 'SIMMODE', 'ENUM_SIMMODE.PID_STEP');
```
# 1. ステップ指令に対する応答のシミュレーション

制御モデルとプラントモデルを開いて確認する。以下のコマンドを実行することで、モデルファイルを開き、Mechanics Explorers でプラントモデルの物理構造を確認することができる。

```matlab
open_system(system_model_name);
set_param(system_model_name, 'SimulationCommand', 'Update');
```

制御対象は板の上に乗っているボールであり、板を傾けることでXY位置を制御することができる。制御量はボールのXY位置、操作量はサーボモーターの指令角度である。PID制御器のゲインはすでに適度に調整されている。


以下のコマンドを実行してモデルの実行結果を確認する。

```matlab
sim(system_model_name);
plot_ball_results_in_SDI;
```
# 2. 円軌道指令に対する応答のシミュレーション

ここで、指令値を円を描くような軌道に変更し、シミュレーションを実行する。

```matlab
set_slddVal('BallAndPlate_system_data.sldd', 'SIMMODE', 'ENUM_SIMMODE.PID_CIRCLE');

open_system(system_model_name);
sim(system_model_name);
Simulink.sdi.view;
Simulink.sdi.clearPreferences;
Simulink.sdi.loadView(SDI_view_path);
```

シミュレーションデータインスペクターに表示されている結果を確認すると、指令値に対して遅れて追従しており、円の半径も指令値より大きくなってしまっている。


MPCを用いることで、この軌道にオフセット無しに追従させたい。


*Copyright 2022 The MathWorks, Inc.*


