# オンライン推定モデルを利用した適応MPCの設計


本サンプルモデルでは、オンラインで逐次推定した状態空間モデルを用いて、適応MPCを実行する制御器の設計方法について紹介する。




モデルをダウンロード、もしくはクローンした後、最初に「rls_mpc_ball_control.prj」をダブルクリックして[プロジェクト](https://jp.mathworks.com/help/matlab/projects.html)を起動すること。


# 必要なツールボックス


本モデルでは、以下のツールボックスを利用する。ただし、インストールしていなくても、モデルを実行せず閲覧するだけであれば可能である。また、例題によっては使わないツールボックスもある。



   -  MATLAB® 
   -  Simulink® 
   -  Stateflow® 
   -  Control System Toolbox™ 
   -  Simulink Control Design™ 
   -  Model Predictive Control Toolbox™ 
   -  System Identification Toolbox™ 
   -  Simscape™, Simscape Multibody™ 

# 概要


モデル予測制御（MPC）は、複雑なシステムにおける高性能な制御機能を実現するための、実用的な手段として注目されている。しかし、MPCの内部モデル設計の工数、また内部モデルと実システムとのずれによる不安定化が課題となっている。




そこで、逐次最小二乗法を用いた実時間モデル推定を行い、そのモデルを内部モデルに適用することで、内部モデル設計の自動化と制御の安定性を向上させる手法を提案する。




最初にPID制御器を用いたバランスボールの位置制御の例を示す。その後、周波数応答推定器を用いてプラントモデルの伝達関数を推定し、そのモデルを内部モデルとして用いたMPCの設計を行う。そして、逐次最小二乗法を用いたオンラインの内部モデル推定を行う手法も紹介する。合わせて、適応MPCを用いた軌道追従制御の例にて、MPCの制御性能を評価する。




また、むだ時間を含むシステムに対して適応MPCを設計する方法も紹介する。


# 目次


[PID制御器によるボールの位置制御](/Design/explain_ball_control_with_PID_md.md)




[周波数応答推定器によるプラントモデルの推定](/Design/run_FRE_and_analyze_frd_data_md.md)




[オンラインモデル推定適応MPCを設計](/Design/design_RLS_MPC_controller_md.md)




[適応MPCによる軌道追従制御](/Design/design_trajectory_control_MPC_md.md)




[むだ時間を含むシステムに対して適応MPCを設計](/Design/design_MPC_with_delay_md.md)


  
# 参考資料


通常のMPCの設計と実装に関しては、以下の資料が参考になる。




[モデル予測制御　設計実装ワークフロー紹介](https://jp.mathworks.com/matlabcentral/fileexchange/77879-mpc-implementation-example/)


  


*Copyright 2022 The MathWorks, Inc.*



