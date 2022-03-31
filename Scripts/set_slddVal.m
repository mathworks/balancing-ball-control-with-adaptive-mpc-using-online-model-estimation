function set_slddVal(sldd_name, val_name, val)
%% Copyright 2022 The MathWorks, Inc.
%%

sldd_obj = Simulink.data.dictionary.open(sldd_name);
data_set_obj = getSection(sldd_obj, 'Design Data');
data_entry = getEntry(data_set_obj, val_name);

valObj = getValue(data_entry);
if isa(valObj, 'Simulink.VariantControl')
    valObj.Value.Value = slexpr(val);
else
    if ischar(val)
        valObj.Value = slexpr(val);
    else
        valObj.Value = val;
    end
end

setValue(data_entry, valObj);

saveChanges(sldd_obj);

end