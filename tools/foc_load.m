function T = foc_load(inFile, fs)
%FOC_LOAD  Unified loader for the FOC analysis tools.
%   Returns a table with a 'time' column plus data columns, from EITHER a
%   UART telemetry CSV (';'/',' , many columns) OR a 20 kHz single-channel
%   binary capture (*.bin). The .bin is wrapped into a one-signal table so
%   the existing CSV-oriented tools run on it unchanged.
if nargin < 2; fs = 20000; end

if endsWith(lower(inFile), '.bin')
    [x, t, ~, ch] = read_foc_bin(inFile, fs);
    switch ch
        case 'speed'; col = 'speed_measured';
        case 'id';    col = 'i_dq_d';
        case 'iq';    col = 'i_dq_q';
        otherwise;    col = 'signal';
    end
    T = table(t, x, 'VariableNames', {'time', col});
else
    opts = detectImportOptions(inFile, 'Delimiter', ';', 'DecimalSeparator', ',');
    opts = setvartype(opts, opts.VariableNames, 'double');
    T = readtable(inFile, opts);
end
end
