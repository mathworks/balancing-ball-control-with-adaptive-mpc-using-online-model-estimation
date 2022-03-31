function plot_ball_results_in_SDI()
%% Copyright 2022 The MathWorks, Inc.
%%
Simulink.sdi.view;
Simulink.sdi.clearAllSubPlots;
Simulink.sdi.clearPreferences;
Simulink.sdi.setSubPlotLayout(2, 2);

%%
RunIDs = Simulink.sdi.getAllRunIDs;
RunID = Simulink.sdi.getRun(RunIDs(end));

plot_single_signal_in_SubPlot(RunID, 'ref.x', 1, 1);
plot_single_signal_in_SubPlot(RunID, 'pos.x', 1, 1);

plot_single_signal_in_SubPlot(RunID, 'ref.y', 2, 1);
plot_single_signal_in_SubPlot(RunID, 'pos.y', 2, 1);

plot_single_signal_in_SubPlot(RunID, 'angle.x', 1, 2);
plot_single_signal_in_SubPlot(RunID, 'angle.y', 2, 2);


end