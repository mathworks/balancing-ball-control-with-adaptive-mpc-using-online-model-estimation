function plot_single_signal_in_SubPlot( ...
    RunID, signal_name, col, row)
%% Copyright 2022 The MathWorks, Inc.
%%
sigID = RunID.getSignalIDsByName(signal_name);
sigHD = RunID.getSignal(sigID(end));
sigHD.plotOnSubPlot(col, row, true);

end