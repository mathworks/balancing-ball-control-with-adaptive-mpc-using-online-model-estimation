function set_slddVal(sldd_name, val_name, val)
%% Copyright 2024 The MathWorks, Inc.
%%
sldd_obj = Simulink.data.connect(sldd_name);
data_obj = sldd_obj.get(val_name);

%%
if isa(data_obj, 'Simulink.VariantControl')
    data_obj.Value.Value = eval(val);
else
    if ischar(val)
        data_obj.Value = slexpr(val);
    else
        data_obj.Value = val;
    end
end

sldd_obj.set(val_name, data_obj);

sldd_obj.saveChanges;

end
