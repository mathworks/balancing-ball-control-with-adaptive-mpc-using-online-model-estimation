function out = get_slddVal(sldd_name, val_name)
%% Copyright 2024 The MathWorks, Inc.
%%

sldd_obj = Simulink.data.connect(sldd_name);
data_obj = sldd_obj.get(val_name);
out = data_obj.Value;

end