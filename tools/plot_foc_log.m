function plot_foc_log(csvFile)
%PLOT_FOC_LOG Plot FOC telemetry logged from the STM32 controller.
%
%   plot_foc_log()          - opens a file picker
%   plot_foc_log(csvFile)   - plots the given CSV log
%
%   Handles the logger's format directly: ';' delimited, ',' as the
%   decimal separator. Column set differs between log versions (e.g.
%   older logs have no duty_cycles, some have current_state instead of
%   hall_debug.combined_state) - every panel is skipped automatically if
%   its columns aren't present in the file.

if nargin < 1 || isempty(csvFile)
    [f, p] = uigetfile({'*.csv', 'CSV log (*.csv)'}, 'Select FOC log file');
    if isequal(f, 0)
        return
    end
    csvFile = fullfile(p, f);
end

T = foc_load(csvFile);

t = getcol(T, 'time');
if isempty(t)
    error('plot_foc_log:noTime', 'No "time" column found in %s', csvFile);
end
t = t - t(1);

dirn   = getcol(T, 'direction');
ang    = getcol(T, 'angle_rad');
da     = getcol(T, 'duty_cycles.a', 'duty_cycles_a');
db     = getcol(T, 'duty_cycles.b', 'duty_cycles_b');
dc     = getcol(T, 'duty_cycles.c', 'duty_cycles_c');
ia     = getcol(T, 'i_abc.a', 'i_abc_a');
ib     = getcol(T, 'i_abc.b', 'i_abc_b');
ic     = getcol(T, 'i_abc.c', 'i_abc_c');
idm    = getcol(T, 'i_dq.d', 'i_dq_d');
iqm    = getcol(T, 'i_dq.q', 'i_dq_q');
idtgt  = getcol(T, 'id_target');
iqtgt  = getcol(T, 'iq_target');
spd    = getcol(T, 'speed_measured');
spdtgt = getcol(T, 'speed_target');
vd     = getcol(T, 'v_dq.d', 'v_dq_d');
vq     = getcol(T, 'v_dq.q', 'v_dq_q');
hallst = getcol(T, 'hall_debug.combined_state', 'hall_debug_combined_state');
h1     = getcol(T, 'hall_debug.h1', 'hall_debug_h1');
h2     = getcol(T, 'hall_debug.h2', 'hall_debug_h2');
h3     = getcol(T, 'hall_debug.h3', 'hall_debug_h3');
motorst = getcol(T, 'current_state'); % FSM state (IDLE/CALIBRATION/RUNNING), only in older logs

fig = figure('Name', csvFile, 'NumberTitle', 'off', 'Color', 'w');
tl = tiledlayout(fig, 'flow', 'TileSpacing', 'compact', 'Padding', 'compact');
[~, fname] = fileparts(csvFile);
title(tl, strrep(fname, '_', '\_'));

axList = gobjects(0);

% --- Speed ---
if ~isempty(spd) || ~isempty(spdtgt)
    ax = nexttile(tl); hold(ax, 'on');
    if ~isempty(spdtgt); plot(ax, t, spdtgt, 'k--', 'DisplayName', 'target'); end
    if ~isempty(spd);    plot(ax, t, spd, 'b-', 'DisplayName', 'measured'); end
    ylabel(ax, 'Speed [RPM]'); title(ax, 'Speed');
    legend(ax, 'show', 'Location', 'best'); grid(ax, 'on');
    axList(end+1) = ax;
end

% --- dq currents ---
if ~isempty(idm) || ~isempty(iqm)
    ax = nexttile(tl); hold(ax, 'on');
    if ~isempty(idtgt); plot(ax, t, idtgt, '--', 'Color', [0.4 0.4 1], 'DisplayName', 'id target'); end
    if ~isempty(idm);   plot(ax, t, idm, 'b-', 'DisplayName', 'id'); end
    if ~isempty(iqtgt); plot(ax, t, iqtgt, '--', 'Color', [1 0.5 0.4], 'DisplayName', 'iq target'); end
    if ~isempty(iqm);   plot(ax, t, iqm, 'r-', 'DisplayName', 'iq'); end
    ylabel(ax, 'Current [A]'); title(ax, 'd/q Currents');
    legend(ax, 'show', 'Location', 'best'); grid(ax, 'on');
    axList(end+1) = ax;
end

% --- abc phase currents ---
if ~isempty(ia) || ~isempty(ib) || ~isempty(ic)
    ax = nexttile(tl); hold(ax, 'on');
    if ~isempty(ia); plot(ax, t, ia, 'DisplayName', 'i_a'); end
    if ~isempty(ib); plot(ax, t, ib, 'DisplayName', 'i_b'); end
    if ~isempty(ic); plot(ax, t, ic, 'DisplayName', 'i_c'); end
    ylabel(ax, 'Current [A]'); title(ax, 'Phase Currents (abc)');
    legend(ax, 'show', 'Location', 'best'); grid(ax, 'on');
    axList(end+1) = ax;
end

% --- dq voltages ---
if ~isempty(vd) || ~isempty(vq)
    ax = nexttile(tl); hold(ax, 'on');
    if ~isempty(vd); plot(ax, t, vd, 'b-', 'DisplayName', 'v_d'); end
    if ~isempty(vq); plot(ax, t, vq, 'r-', 'DisplayName', 'v_q'); end
    if ~isempty(vd) && ~isempty(vq)
        plot(ax, t, hypot(vd, vq), 'k:', 'DisplayName', '|v|');
    end
    ylabel(ax, 'Voltage [V]'); title(ax, 'd/q Voltages');
    legend(ax, 'show', 'Location', 'best'); grid(ax, 'on');
    axList(end+1) = ax;
end

% --- electrical angle ---
if ~isempty(ang)
    ax = nexttile(tl);
    plot(ax, t, ang, '-', 'MarkerSize', 3);
    ylabel(ax, 'Angle [rad]'); ylim(ax, [0, 2*pi]);
    title(ax, 'Electrical Angle (PLL)'); grid(ax, 'on');
    axList(end+1) = ax;
end

% --- duty cycles ---
if ~isempty(da) || ~isempty(db) || ~isempty(dc)
    ax = nexttile(tl); hold(ax, 'on');
    if ~isempty(da); plot(ax, t, da, 'DisplayName', 'duty_a'); end
    if ~isempty(db); plot(ax, t, db, 'DisplayName', 'duty_b'); end
    if ~isempty(dc); plot(ax, t, dc, 'DisplayName', 'duty_c'); end
    yline(ax, 0.5, 'k:', 'DisplayName', 'center');
    ylabel(ax, 'Duty [0-1]'); ylim(ax, [0, 1]);
    title(ax, 'PWM Duty Cycles'); legend(ax, 'show', 'Location', 'best'); grid(ax, 'on');
    axList(end+1) = ax;
end

% --- combined hall state (0-7) ---
if ~isempty(hallst)
    ax = nexttile(tl);
    stairs(ax, t, hallst, 'k-', 'LineWidth', 1.2);
    ylabel(ax, 'Hall state'); ylim(ax, [-0.5, 7.5]); yticks(ax, 0:7);
    title(ax, 'Hall State (combined)'); grid(ax, 'on');
    axList(end+1) = ax;
end

% --- raw hall bits, logic-analyzer style (each trace offset for clarity) ---
if ~isempty(h1) || ~isempty(h2) || ~isempty(h3)
    ax = nexttile(tl); hold(ax, 'on');
    if ~isempty(h1); stairs(ax, t, h1 + 4, 'DisplayName', 'h1'); end
    if ~isempty(h2); stairs(ax, t, h2 + 2, 'DisplayName', 'h2'); end
    if ~isempty(h3); stairs(ax, t, h3 + 0, 'DisplayName', 'h3'); end
    ylabel(ax, 'Hall bits (offset)'); ylim(ax, [-0.5, 5.5]);
    yticks(ax, [0 1 2 3 4 5]); yticklabels(ax, {'0', '1', '0', '1', '0', '1'});
    title(ax, 'Hall Bits (h3 bottom, h2 mid, h1 top)'); legend(ax, 'show', 'Location', 'best'); grid(ax, 'on');
    axList(end+1) = ax;
end

if ~isempty(dirn)
    ax = nexttile(tl);
    stairs(ax, t, dirn, 'm-', 'LineWidth', 1.2);
    ylabel(ax, 'Direction'); ylim(ax, [-1.5, 1.5]);
    title(ax, 'Rotation Direction'); grid(ax, 'on');
    axList(end+1) = ax;
end

% --- FSM state (older logs only: 0=IDLE, 1=CALIBRATION, 2=RUNNING) ---
if ~isempty(motorst)
    ax = nexttile(tl);
    stairs(ax, t, motorst, 'Color', [0.2 0.6 0.2], 'LineWidth', 1.2);
    ylabel(ax, 'State'); ylim(ax, [-0.5, 2.5]);
    yticks(ax, [0 1 2]); yticklabels(ax, {'IDLE', 'CALIB', 'RUNNING'});
    title(ax, 'Motor State'); grid(ax, 'on');
    axList(end+1) = ax;
end

xlabel(tl, 'Time [s]');
if ~isempty(axList)
    linkaxes(axList, 'x');
    xlim(axList(1), [t(1), t(end)]);
end

end

function col = getcol(T, varargin)
%GETCOL Fetch a column from T by matching any of the given name patterns
%   against the table's variable names (exact match first, then
%   substring match, both case-insensitive). Returns [] if none match.
vn = T.Properties.VariableNames;
col = [];
for k = 1:numel(varargin)
    pat = varargin{k};
    idx = find(strcmpi(vn, pat), 1);
    if isempty(idx)
        idx = find(contains(vn, pat, 'IgnoreCase', true), 1);
    end
    if ~isempty(idx)
        col = T{:, idx};
        return
    end
end
end
