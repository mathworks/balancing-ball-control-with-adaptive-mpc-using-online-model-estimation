
# 適応MPCによる軌道追従制御
# 初期化
```matlab
system_model_name = 'BallAndPlate_system';
ampc_controller_name = 'BallAndPlate_AMPC_Controller';
SDI_view_path = which('circle_move_XY_view.mldatx');

set_slddVal('BallAndPlate_system_data.sldd', 'DELAYMODE', 'ENUM_DELAYMODE.OFF');
set_slddVal('BallAndPlate_system_data.sldd', 'SIMMODE', 'ENUM_SIMMODE.MPC_A_TR');
```

ここでは、前回設計した適応MPCを用いてボールの円軌道追従制御を設計する。


MPCオブジェクトを読み込む。

```matlab
load('AMPC_Controller_design_data.mat');
```

MPCオブジェクトの設定を調整する。

```matlab
% 予測ホライズンと制御ホライズン
mpcObj.PredictionHorizon = 30;
mpcObj.ControlHorizon = 2;
set_slddVal('BallAndPlate_system_data.sldd', ...
    'MPC_PredictionHorizon', mpcObj.PredictionHorizon);
```

予測ホライズンの長さによって指令値に対する追従性が変化する。


MPCは予測ホライズンの長さ分だけ、将来の指令値（つまり指令値の軌道）を入力することができる。これにより、MPCは指令値軌道に対して遅れなしに追従することが可能となる。


モデルを開いて確認する。

```matlab
open_system(system_model_name);
```

また、モデル化誤差やカルマンフィルターの応答遅れの影響によりオフセットが生じるため、ここで積分ゲインを入力しておく。

```matlab
set_slddVal('BallAndPlate_system_data.sldd', 'offset_free_I_gain', 0.01);
```

モデルを実行し、制御結果を確認する。

```matlab
sim(system_model_name);
```

```matlabTextOutput
   測定出力チャネル #1 に外乱が追加されていないと仮定します。
   測定出力チャネル #2 に外乱が追加されていないと仮定します。
-->"Model.Noise" プロパティが空です。それぞれの測定出力にホワイト ノイズを仮定します。
```

```matlab
Simulink.sdi.view;
Simulink.sdi.clearPreferences;
Simulink.sdi.loadView(SDI_view_path);
```

円軌道に対して、遅れなく追従できていることがわかる。


最後にパラメーターを戻す。

```matlab
set_slddVal('BallAndPlate_system_data.sldd', 'offset_free_I_gain', 0);
```


*Copyright 2022 The MathWorks, Inc.*


