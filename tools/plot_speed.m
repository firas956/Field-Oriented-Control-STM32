% plot_speed.m
% Extract and plot measured vs reference (target) speed from the newest
% STM32CubeMonitor "Log_myVariables_*.csv" export.
%
% The CSV is European-formatted:
%   - column delimiter : ';'
%   - decimal separator: ','   (also used inside 1.2e-3 style exponents)
% so we convert every ',' to '.' before parsing (safe: ',' is never a delimiter).
%
% The resulting figure is interactive: use the toolbar magnifier or draw a
% zoom box to inspect the speed transients / steady state.

clear; clc; 

% ------------------------------------------------------------------- config
% Look for the logs in this script's own folder (falls back to pwd).
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end

% Automatically pick the most recent Log_myVariables_*.csv.
d = dir(fullfile(here, 'speed_after_compensation.csv'));

[~, idx] = max([d.datenum]);
csvFile  = fullfile(d(idx).folder, d(idx).name);
fprintf('Reading: %s\n', d(idx).name);

% -------------------------------------------------- read + fix decimal comma
raw = fileread(csvFile);
raw = strrep(raw, ',', '.');          % decimal comma -> dot

tmp = [tempname '.csv'];
fid = fopen(tmp, 'w'); fwrite(fid, raw); fclose(fid);
cleanup = onCleanup(@() delete(tmp)); %#ok<NASGU>  remove temp file on exit

opts = detectImportOptions(tmp, 'Delimiter', ';');
T = readtable(tmp, opts);

% -------------------------------------------------------------- extract data
% Reference by fixed column index (order is set by the CubeMonitor config):
%   col 1  = time
%   col 14 = foc_core.speed_measured
%   col 15 = foc_core.speed_target
t         = T{:, 1};
speedMeas = T{:, 14};
speedRef  = T{:, 15};

% -------------------------------------------------------------------- plot
figure('Name', 'FOC speed tracking', 'Color', 'w');
plot(t, speedRef,  'r--', 'LineWidth', 1.5); hold on;
plot(t, speedMeas, 'b-',  'LineWidth', 1.0);
grid on; box on;
xlabel('Time [s]');
ylabel('Speed');
title('FOC speed tracking: measured vs reference');
legend('Reference (speed\_target)', 'Measured (speed\_measured)', ...
       'Location', 'best');

% Turn on interactive zoom right away.
zoom on;
