
# オンラインモデル推定適応MPCを設計
# 初期化
```matlab
system_model_name = 'BallAndPlate_system';
ampc_controller_name = 'BallAndPlate_AMPC_Controller';
rlsmpc_controller_name = 'BallAndPlate_RLSMPC_Controller';

Ts = get_slddVal('BallAndPlate_system_data.sldd', 'TimeStep_MPC');

set_slddVal('BallAndPlate_system_data.sldd', 'DELAYMODE', 'ENUM_DELAYMODE.OFF');
set_slddVal('BallAndPlate_system_data.sldd', 'SIMMODE', 'ENUM_SIMMODE.MPC_A');
```
# 適応MPCを設計

前回得られた伝達関数のプラントモデルをMPCの内部モデルとして用いて適応MPCを設計する。最初にオンライン推定を行わない形のMPCを設計し、その後オンライン推定を行う形のMPCを設計する。


伝達関数のプラントモデルを離散時間の状態空間モデルに変換する。

```matlab
load('fre_result.mat');

BallAndPlate_1aixs_tf_d = c2d(estimated_BallAndPlate_transfer_function, Ts)
```

```matlabTextOutput
BallAndPlate_1aixs_tf_d =
 
  0.0008305 z^3 - 0.001327 z^2 + 0.0006849 z + 0.0001172
  ------------------------------------------------------
    z^4 - 2.613 z^3 + 2.227 z^2 - 0.6146 z + 0.0007818
 
サンプル時間:  0.02 seconds
離散時間の伝達関数です。
```

ここで、X軸とY軸は独立しており、同一の物理構造であるため、同じ伝達関数を適用する。

```matlab
BallAndPlate_ss_d = ss([BallAndPlate_1aixs_tf_d, 0; 0, BallAndPlate_1aixs_tf_d])
```

```matlabTextOutput
BallAndPlate_ss_d =
 
  A = 
             x1        x2        x3        x4        x5        x6        x7        x8
   x1     2.613    -1.113    0.6146  -0.02502         0         0         0         0
   x2         2         0         0         0         0         0         0         0
   x3         0       0.5         0         0         0         0         0         0
   x4         0         0   0.03125         0         0         0         0         0
   x5         0         0         0         0     2.613    -1.113    0.6146  -0.02502
   x6         0         0         0         0         2         0         0         0
   x7         0         0         0         0         0       0.5         0         0
   x8         0         0         0         0         0         0   0.03125         0
 
  B = 
           u1      u2
   x1  0.0625       0
   x2       0       0
   x3       0       0
   x4       0       0
   x5       0  0.0625
   x6       0       0
   x7       0       0
   x8       0       0
 
  C = 
             x1        x2        x3        x4        x5        x6        x7        x8
   y1   0.01329  -0.01061   0.01096   0.06001         0         0         0         0
   y2         0         0         0         0   0.01329  -0.01061   0.01096   0.06001
 
  D = 
       u1  u2
   y1   0   0
   y2   0   0
 
サンプル時間:  0.02 seconds
離散時間状態空間モデル。
```

ここで、カルマンフィルターを構成するための変数の値を更新する。

```matlab
set_slddVal('BallAndPlate_system_data.sldd', 'EKF_x_num', size(BallAndPlate_ss_d.A, 1));
set_slddVal('BallAndPlate_system_data.sldd', 'EKF_u_num', size(BallAndPlate_ss_d.B, 2));
set_slddVal('BallAndPlate_system_data.sldd', 'EKF_y_num', size(BallAndPlate_ss_d.C, 1));
```

MPCの内部モデルの状態は全て観測できるわけではないので、カルマンフィルターを用いて推定を行っている。今回は「Adaptive MPC Controller」ブロックから独立して設計を行っている。オンライン推定を行うことを考慮して、拡張カルマンフィルター（EKF）を用いて設計する。


以下のコマンドを実行してモデルを確認する。「EKF\_for\_BallAndPlate\_ss」サブシステムにてカルマンフィルターを設計している。

```matlab
open_system(ampc_controller_name);
```

MPCのオブジェクトを構築する。

```matlab
mpcObj = mpc(BallAndPlate_ss_d, Ts);
```

```matlabTextOutput
-->"PredictionHorizon" プロパティが空です。既定の 10 を仮定します。
-->"ControlHorizon" プロパティが空です。既定の 2 を仮定します。
-->"Weights.ManipulatedVariables" プロパティが空です。既定の 0.00000 を仮定します。
-->"Weights.ManipulatedVariablesRate" プロパティが空です。既定の 0.10000 を仮定します。
-->"Weights.OutputVariables" プロパティが空です。既定の 1.00000 を仮定します。
```

```matlab
% 予測ホライズンと制御ホライズン
mpcObj.PredictionHorizon = 60;
mpcObj.ControlHorizon = 4;
set_slddVal('BallAndPlate_system_data.sldd', ...
    'MPC_PredictionHorizon', mpcObj.PredictionHorizon);

% カルマンフィルターを外部で設計するため、以下を設定する必要がある。
% また、ブロック側も
% 「組み込みカルマンフィルターの代わりにカスタム状態推定を使用する」
% にチェックを入れること。
setEstimator(mpcObj, 'custom');

% サーボ角度指令値の上下限制約 
mpcObj.ManipulatedVariables(1).Max = 90;
mpcObj.ManipulatedVariables(2).Max = 90;
mpcObj.ManipulatedVariables(1).Min = -90;
mpcObj.ManipulatedVariables(2).Min = -90;

% 最適化の重み
mpcObj.Weights.OutputVariables = [2, 2];
mpcObj.Weights.ManipulatedVariables = [0.1, 0.1];
mpcObj.Weights.ManipulatedVariablesRate = [0, 0];

save(fullfile(pjObj.RootFolder, 'Data', 'AMPC_Controller_design_data.mat'), ...
    'BallAndPlate_ss_d', 'mpcObj');
```

モデルを実行し、制御結果を確認する。

```matlab
open_system(system_model_name);
sim(system_model_name);
```

```matlabTextOutput
   測定出力チャネル #1 に外乱が追加されていないと仮定します。
   測定出力チャネル #2 に外乱が追加されていないと仮定します。
-->"Model.Noise" プロパティが空です。それぞれの測定出力にホワイト ノイズを仮定します。
```

```matlab
plot_ball_results_in_SDI;
```

指令値に追従できており、また操作量の上下限制約を守って制御を行っていることが確認できる。

# オンラインモデル推定適応MPCを設計

初期化

```matlab
set_slddVal('BallAndPlate_system_data.sldd', 'SIMMODE', 'ENUM_SIMMODE.MPC_ARLS');
```

ここでは、上記で設計した適用MPCを拡張し、オンラインモデル推定をできるようにする。


オンライン推定の手法として、逐次最小二乗法を用いる。System Identification Toolbox™ の「Recursive Polynomial Model Estimator」ブロックを用いてARXモデルを推定し、「Model Type Converter」ブロックにより状態空間モデルに変換する。


以下のコマンドによりテストモデルを開いて確認する。

```matlab
open_system('check_RLS_estimated_model');
```

前回得られた伝達関数のプラントモデル（BallAndPlate\_1aixs\_tf\_d）は、「Recursive Polynomial Model Estimator」ブロックの初期値として用いることができるが、伝達関数、ARX、状態空間と変換される過程で BallAndPlate\_ss\_d とは異なる状態空間モデルになってしまう。


よって、まずこの状態空間モデルを「check\_RLS\_estimated\_model.slx」を実行することで求め、それを用いてMPCオブジェクトの設計を行うこととする。

```matlab
simout = sim('check_RLS_estimated_model');
sys_A = simout.logsout.get('<A>').Values.Data;
sys_B = simout.logsout.get('<B>').Values.Data;
sys_C = simout.logsout.get('<C>').Values.Data;
sys_D = simout.logsout.get('<D>').Values.Data;

BallAndPlate_d_arx = ss(sys_A, sys_B, sys_C, sys_D, Ts)
```

```matlabTextOutput
BallAndPlate_d_arx =
 
  A = 
               x1          x2          x3          x4          x5          x6          x7          x8
   x1       2.613           1           0           0           0           0           0           0
   x2      -2.227           0           1           0           0           0           0           0
   x3      0.6146           0           0           1           0           0           0           0
   x4  -0.0007818           0           0           0           0           0           0           0
   x5           0           0           0           0       2.613           1           0           0
   x6           0           0           0           0      -2.227           0           1           0
   x7           0           0           0           0      0.6146           0           0           1
   x8           0           0           0           0  -0.0007818           0           0           0
 
  B = 
              u1         u2
   x1  0.0008305          0
   x2  -0.001327          0
   x3  0.0006849          0
   x4  0.0001172          0
   x5          0  0.0008305
   x6          0  -0.001327
   x7          0  0.0006849
   x8          0  0.0001172
 
  C = 
       x1  x2  x3  x4  x5  x6  x7  x8
   y1   1   0   0   0   0   0   0   0
   y2   0   0   0   0   1   0   0   0
 
  D = 
       u1  u2
   y1   0   0
   y2   0   0
 
サンプル時間:  0.02 seconds
離散時間状態空間モデル。
```

MPCオブジェクトを設計する。

```matlab
mpcObj_RLS = mpc(BallAndPlate_d_arx, Ts);
```

```matlabTextOutput
-->"PredictionHorizon" プロパティが空です。既定の 10 を仮定します。
-->"ControlHorizon" プロパティが空です。既定の 2 を仮定します。
-->"Weights.ManipulatedVariables" プロパティが空です。既定の 0.00000 を仮定します。
-->"Weights.ManipulatedVariablesRate" プロパティが空です。既定の 0.10000 を仮定します。
-->"Weights.OutputVariables" プロパティが空です。既定の 1.00000 を仮定します。
```

```matlab
% 予測ホライズンと制御ホライズン
mpcObj_RLS.PredictionHorizon = 60;
mpcObj_RLS.ControlHorizon = 4;
set_slddVal('BallAndPlate_system_data.sldd', ...
    'MPC_PredictionHorizon', mpcObj_RLS.PredictionHorizon);

% カルマンフィルターを外部で設計する設定
setEstimator(mpcObj_RLS, 'custom');
% ここで、自動的に出力外乱モデルが挿入されてしまうため、そのモデルを削除する。
setoutdist(mpcObj_RLS, 'model', tf(zeros(size(BallAndPlate_d_arx.C, 1), 1)))

% サーボ角度指令値の上下限制約 
mpcObj_RLS.ManipulatedVariables(1).Max = 90;
mpcObj_RLS.ManipulatedVariables(2).Max = 90;
mpcObj_RLS.ManipulatedVariables(1).Min = -90;
mpcObj_RLS.ManipulatedVariables(2).Min = -90;

% 最適化の重み
mpcObj_RLS.Weights.OutputVariables = [2, 2];
mpcObj_RLS.Weights.ManipulatedVariables = [0.1, 0.1];
mpcObj_RLS.Weights.ManipulatedVariablesRate = [0, 0];

save(fullfile(pjObj.RootFolder, 'Data', 'RLSMPC_Controller_design_data.mat'), ...
    'BallAndPlate_d_arx', 'mpcObj_RLS');
```

モデルを実行し、制御結果を確認する。

```matlab
open_system(system_model_name);
sim(system_model_name);
```

```matlabTextOutput
-->"Model.Noise" プロパティが空です。それぞれの測定出力にホワイト ノイズを仮定します。
```

```matlab
plot_ball_results_in_SDI;
```
# モデル化誤差に対する適応能力の確認

オンラインモデル推定適応MPCは、内部モデルと実際のプラントモデルとの間にモデル化誤差があった場合に、制御をしながら内部モデルを修正し、制御を安定化できることが期待される。そこで、敢えてプラントモデルの物理パラメーターを変更し、それに対して適応出来るかどうかを確認する。


以下のコマンドを実行し、パラメーターを変更する。以下では、X位置を制御する方のサーボモーターのシャフトの減衰係数を変更している。また、適応過程を確認するため、シミュレーション時間を180秒に変更する。

```matlab
set_slddVal('BallAndPlate_system_data.sldd', 'motor_shaft_mu_1', 10);
set_slddVal('BallAndPlate_system_data.sldd', 'StopTime', 180);
```

モデルを実行し、制御結果を確認する。

```matlab
open_system(system_model_name);
sim(system_model_name);
plot_ball_results_in_SDI;
```

時間の経過と共に振動的な応答波形が緩和されていく様子が確認できる。


しかし同時に、指令値に対する定常偏差も増加している。これは、定常的な面ではモデル化誤差が拡大したためと考えられる。オフセットを除去するため、本モデルには積分器も用意している（「RLS\_offset\_free」サブシステム）。「offset\_free\_I\_gain」を増加させることで、定常偏差を減らすことができる。


設定を元に戻す。

```matlab
set_slddVal('BallAndPlate_system_data.sldd', 'motor_shaft_mu_1', 0);
set_slddVal('BallAndPlate_system_data.sldd', 'StopTime', 11.99);
```


*Copyright 2022 The MathWorks, Inc.*


