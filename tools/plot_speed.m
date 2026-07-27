% plot_speed.m
% Extract and plot measured vs reference (target) speed from an
% STM32CubeMonitor CSV export.
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
% DATASET picks which capture to plot:
%   0 -> the original capture. Columns are addressed by FIXED INDEX, matching
%        the CubeMonitor variable list as it was when that log was taken.
%   1 -> a newer capture. Columns are resolved BY NAME from the header, so
%        adding or reordering variables in CubeMonitor cannot silently plot
%        the wrong signal. Falls back to the indices below if the names are
%        not found (e.g. an export with generic Var1..VarN headers).
DATASET = 1;

switch DATASET
    case 0
        cfg.file     = 'step_response_with12.csv';
        cfg.byName   = false;
        cfg.colMeas  = 14;
        cfg.colRef   = 15;
        cfg.tag      = 'before harmonic trim';

    case 1
        cfg.file     = 'step_response_without_12.csv';   % <- new export goes here
        cfg.byName   = true;
        cfg.nameMeas = 'speed_measured';
        cfg.nameRef  = 'speed_target';
        cfg.colMeas  = 14;      % fallback only
        cfg.colRef   = 15;      % fallback only
        cfg.tag      = 'with 6th-harmonic trim';

    otherwise
        error('plot_speed:badDataset', 'DATASET must be 0 or 1, got %g.', DATASET);
end

% Look for the logs in this script's own folder (falls back to pwd).
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end

% cfg.file may be an exact name or a wildcard; a wildcard takes the newest hit.
d = dir(fullfile(here, cfg.file));
if isempty(d)
    error('plot_speed:noFile', ...
          'No file matching "%s" in %s\nSet cfg.file for DATASET = %d.', ...
          cfg.file, here, DATASET);
end
[~, idx] = max([d.datenum]);
csvFile  = fullfile(d(idx).folder, d(idx).name);
fprintf('Reading: %s  (DATASET = %d, %s)\n', d(idx).name, DATASET, cfg.tag);

% -------------------------------------------------- read + fix decimal comma
raw = fileread(csvFile);
raw = strrep(raw, ',', '.');          % decimal comma -> dot

tmp = [tempname '.csv'];
fid = fopen(tmp, 'w'); fwrite(fid, raw); fclose(fid);
cleanup = onCleanup(@() delete(tmp)); %#ok<NASGU>  remove temp file on exit

opts = detectImportOptions(tmp, 'Delimiter', ';');
T = readtable(tmp, opts);

% -------------------------------------------------------------- extract data
% Column 1 is always the CubeMonitor timestamp.
if cfg.byName
    colMeas = pickColumn(T, cfg.nameMeas, cfg.colMeas);
    colRef  = pickColumn(T, cfg.nameRef,  cfg.colRef);
else
    colMeas = cfg.colMeas;
    colRef  = cfg.colRef;
end

nCol = width(T);
if max(colMeas, colRef) > nCol
    error('plot_speed:badColumn', ...
          'Need column %d but the file has only %d. Fix cfg for DATASET = %d.', ...
          max(colMeas, colRef), nCol, DATASET);
end

t         = T{:, 1};
speedMeas = T{:, colMeas};
speedRef  = T{:, colRef};

% -------------------------------------------------------------------- plot
figure('Name', sprintf('FOC speed tracking (%s)', cfg.tag), 'Color', 'w');
plot(t, speedRef,  'r--', 'LineWidth', 1.5); hold on;
plot(t, speedMeas, 'b-',  'LineWidth', 1.0);
grid on; box on;
xlabel('Time [s]');
ylabel('Speed [rpm]');
title(sprintf('FOC speed tracking: measured vs reference (%s)', cfg.tag));
legend('Reference (speed\_target)', 'Measured (speed\_measured)', ...
       'Location', 'best');

% Turn on interactive zoom right away.
zoom on;

% ------------------------------------------------------------------ helpers
function col = pickColumn(T, wanted, fallback)
% Find the column whose header contains 'wanted'. readtable mangles the
% CubeMonitor headers ('foc_core.speed_measured' -> 'foc_core_speed_measured'),
% so match on a substring rather than the full name. Ambiguity is an error:
% 'speed_target' would also match 'speed_target_dot' if both are logged.
    names = T.Properties.VariableNames;
    hit   = find(contains(names, wanted));

    if isempty(hit)
        warning('plot_speed:nameNotFound', ...
                'No column matching "%s"; falling back to index %d (%s).', ...
                wanted, fallback, names{fallback});
        col = fallback;
        return;
    end

    if numel(hit) > 1
        % Prefer an exact-suffix match before giving up.
        exact = hit(endsWith(names(hit), wanted));
        if numel(exact) == 1
            col = exact;
            return;
        end
        error('plot_speed:ambiguousName', ...
              '"%s" matches %d columns: %s. Use DATASET = 0 style fixed indices.', ...
              wanted, numel(hit), strjoin(names(hit), ', '));
    end

    col = hit;
end
