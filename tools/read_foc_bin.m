function [x, t, fs, channel] = read_foc_bin(binFile, fs)
%READ_FOC_BIN  Load a single-channel float32 capture dumped from the STM32.
%
%   [x, t, fs, channel] = read_foc_bin(binFile)       % fs = 20000 Hz
%   [x, t, fs, channel] = read_foc_bin(binFile, fs)
%
%   The firmware one-shot datalogger fills datalog_buf[] with ONE channel
%   (speed_measured, i_d OR i_q - picked by uncommenting a line in
%   MotorControl_RunIteration) at the 20 kHz ISR rate. Extract with:
%       (gdb) dump binary memory speed_20k.bin &datalog_buf[0] &datalog_buf[28672]
%   -> raw little-endian float32, uniformly sampled, no header/timestamps.
%
%   CHANNEL is inferred from the file name (speed|id|iq) for labeling only.
%   Name dumps accordingly, e.g. iq_2000rpm.bin.

if nargin < 2 || isempty(fs); fs = 20000; end
if nargin < 1 || isempty(binFile)
    [f, p] = uigetfile({'*.bin','Binary capture (*.bin)'}, 'Select FOC capture');
    if isequal(f, 0); x = []; t = []; channel = ''; return; end
    binFile = fullfile(p, f);
end

fid = fopen(binFile, 'r', 'ieee-le');    % STM32F4 + x86 both little-endian
if fid < 0; error('read_foc_bin:open', 'cannot open %s', binFile); end
x = fread(fid, inf, 'single=>double');   % float32 on target -> double here
fclose(fid);
x = x(:);
if isempty(x); error('read_foc_bin:empty', '%s has no samples', binFile); end

t = (0:numel(x)-1).' / fs;
channel = infer_channel(binFile);
fprintf('read %d samples (%.3f s @ %.0f Hz) from %s  [channel: %s]\n', ...
        numel(x), t(end), fs, binFile, channel);
end

function ch = infer_channel(f)
[~, name] = fileparts(lower(f));
if     contains(name, 'speed');                       ch = 'speed';
elseif contains(name, 'iq') || contains(name, 'i_q'); ch = 'iq';
elseif contains(name, 'id') || contains(name, 'i_d'); ch = 'id';
else;                                                 ch = 'signal';
end
end
