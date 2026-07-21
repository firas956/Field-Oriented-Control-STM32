function S = foc_steadystate_analysis(csvFile, window)
%FOC_STEADYSTATE_ANALYSIS  Mean / variance characterization of a FOC log.
%
%   S = foc_steadystate_analysis()                 - file picker, auto window
%   S = foc_steadystate_analysis(csvFile)           - auto steady-state window
%   S = foc_steadystate_analysis(csvFile,[t0 t1])   - explicit window [s]
%
%   Reports, over a constant-operating-point window:
%     - MEAN of the regulated signals  -> steady-state tracking accuracy
%       (mean - target = steady-state error; ~0 for a healthy PI loop)
%     - VARIANCE / STD                 -> ripple / noise floor at fixed setpoint
%     - SNR (|mean|/std)               -> a scalar "how clean" figure of merit
%
%   Signals analysed: i_dq.d, i_dq.q, speed_measured, v_dq.d, v_dq.q.
%   Phase currents i_abc are AC at steady state (mean ~ 0), so their RMS is
%   reported instead of a meaningless mean/variance.
%
%   The returned struct S holds every number so you can tabulate it in a
%   report. A figure shows each signal over the window with its mean and
%   +/-1 sigma / +/-2 sigma bands, plus a histogram of the deviation.
%
%   NOTE: this assumes the logger's telemetry rate (~300-500 Hz). It
%   characterizes the closed-loop regulation, NOT wideband ripple - the
%   fast harmonics (e.g. 6*f_e torque ripple) are aliased at this rate and
%   need the FFT of an ISR-rate capture, done separately.

if nargin < 1 || isempty(csvFile)
    [f, p] = uigetfile({'*.csv','CSV log (*.csv)'}, 'Select FOC log file');
    if isequal(f, 0); S = []; return; end
    csvFile = fullfile(p, f);
end
if nargin < 2; window = []; end

% ---- load (CSV telemetry or 20 kHz single-channel .bin) ----
T = foc_load(csvFile);

t = getcol(T, 'time');
if isempty(t); error('foc_ss:noTime', 'No "time" column in %s', csvFile); end
t = t - t(1);

spd    = getcol(T, 'speed_measured');
spdtgt = getcol(T, 'speed_target');
idm    = getcol(T, 'i_dq.d', 'i_dq_d');
iqm    = getcol(T, 'i_dq.q', 'i_dq_q');
idtgt  = getcol(T, 'id_target');
iqtgt  = getcol(T, 'iq_target');
vd     = getcol(T, 'v_dq.d', 'v_dq_d');
vq     = getcol(T, 'v_dq.q', 'v_dq_q');
ia     = getcol(T, 'i_abc.a', 'i_abc_a');
ib     = getcol(T, 'i_abc.b', 'i_abc_b');
ic     = getcol(T, 'i_abc.c', 'i_abc_c');

% ---- choose the steady-state window ----
isBin = endsWith(lower(csvFile), '.bin');
if ~isempty(window)
    idx = t >= window(1) & t <= window(2);
    how = sprintf('user window [%.3f, %.3f] s', window(1), window(2));
elseif isBin
    idx = true(size(t));
    how = 'full capture (.bin, steady-state by construction)';
else
    [idx, how] = auto_window(t, spd, spdtgt);
end
if nnz(idx) < 10
    error('foc_ss:shortWindow', 'Steady-state window too short (%d samples). Pass an explicit [t0 t1].', nnz(idx));
end
tw = t(idx);

fprintf('\n=== FOC steady-state analysis ===\n');
fprintf('File   : %s\n', csvFile);
fprintf('Window : %s  ->  %d samples, %.3f s\n', how, nnz(idx), tw(end)-tw(1));
fprintf('%-14s %10s %10s %10s %12s %12s %8s\n', ...
        'signal','target','mean','std','variance','ss_error','SNR');
fprintf('%s\n', repmat('-', 1, 80));

S = struct('file', csvFile, 'window', [tw(1) tw(end)], 'n', nnz(idx));

% regulated DC signals: mean/var meaningful
S.speed = report('speed[RPM]',  spd, spdtgt, idx);
S.id    = report('i_d[A]',      idm, idtgt,  idx);
S.iq    = report('i_q[A]',      iqm, iqtgt,  idx);
S.vd    = report('v_d[V]',      vd,  [],     idx);
S.vq    = report('v_q[V]',      vq,  [],     idx);

% phase currents: AC -> report RMS (mean/var of a sinusoid is misleading)
fprintf('%s\n', repmat('-', 1, 80));
S.ia_rms = report_rms('i_a[A]', ia, idx);
S.ib_rms = report_rms('i_b[A]', ib, idx);
S.ic_rms = report_rms('i_c[A]', ic, idx);
fprintf('(phase currents are AC at steady state -> RMS shown, not mean/var)\n\n');

% ---- figure: each regulated signal with mean +/- sigma bands + histogram ----
plot_signals = { 'speed_{meas} [RPM]', spd, spdtgt; ...
                 'i_d [A]',           idm, idtgt; ...
                 'i_q [A]',           iqm, iqtgt; ...
                 'v_q [V]',           vq,  [] };
plot_signals = plot_signals(~cellfun(@(x) isempty(x), plot_signals(:,2)), :);
nP = size(plot_signals, 1);
if nP > 0
    fig = figure('Name', ['Steady-state: ' csvFile], 'NumberTitle', 'off', 'Color', 'w');
    tl = tiledlayout(fig, nP, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    [~, fn] = fileparts(csvFile);
    title(tl, {strrep(fn,'_','\_'), sprintf('steady-state window: %.3f - %.3f s', tw(1), tw(end))});
    for k = 1:nP
        name = plot_signals{k,1}; y = plot_signals{k,2}; ytg = plot_signals{k,3};
        yw = y(idx); m = mean(yw); s = std(yw);
        % time series with bands
        ax = nexttile(tl); hold(ax,'on');
        fill(ax, [tw; flipud(tw)], [m-2*s+0*tw; flipud(m+2*s+0*tw)], ...
             [0.9 0.9 0.95], 'EdgeColor','none', 'DisplayName','\pm2\sigma');
        fill(ax, [tw; flipud(tw)], [m-s+0*tw;  flipud(m+s+0*tw)], ...
             [0.75 0.8 0.95], 'EdgeColor','none', 'DisplayName','\pm1\sigma');
        plot(ax, tw, yw, 'b-', 'DisplayName','signal');
        yline(ax, m, 'k-', 'LineWidth',1.2, 'DisplayName','mean');
        if ~isempty(ytg); yline(ax, mean(ytg(idx)), 'r--', 'LineWidth',1.2, 'DisplayName','target'); end
        ylabel(ax, name); grid(ax,'on');
        if k==1; legend(ax,'show','Location','best'); end
        if k==nP; xlabel(ax,'Time [s]'); end
        % histogram of deviation from mean
        ax2 = nexttile(tl);
        histogram(ax2, yw - m, 30, 'FaceColor',[0.3 0.5 0.9], 'EdgeColor','none');
        xlabel(ax2, sprintf('%s - mean   (\\sigma=%.3g)', name, s)); ylabel(ax2,'count');
        grid(ax2,'on');
    end
end
end

% ======================================================================
function st = report(name, y, tgt, idx)
if isempty(y)
    st = struct('name',name,'target',NaN,'mean',NaN,'std',NaN,'var',NaN,'ss_error',NaN,'snr',NaN);
    return;
end
yw = y(idx);
m = mean(yw); s = std(yw); v = var(yw);
if ~isempty(tgt)
    tg = mean(tgt(idx)); err = m - tg;
else
    tg = NaN; err = NaN;
end
snr = abs(m) / s;   % |mean|/std
st = struct('name',name,'target',tg,'mean',m,'std',s,'var',v,'ss_error',err,'snr',snr);
if isnan(tg)
    fprintf('%-14s %10s %10.4g %10.4g %12.4g %12s %8.2f\n', name,'--',m,s,v,'--',snr);
else
    fprintf('%-14s %10.4g %10.4g %10.4g %12.4g %12.4g %8.2f\n', name,tg,m,s,v,err,snr);
end
end

function st = report_rms(name, y, idx)
if isempty(y); st = struct('name',name,'rms',NaN,'mean',NaN); return; end
yw = y(idx);
r = sqrt(mean(yw.^2)); m = mean(yw);
st = struct('name',name,'rms',r,'mean',m);
fprintf('%-14s %10s %10.4g %10s %12s %12s %8s   (RMS=%.4g)\n', name,'--',m,'','','','', r);
end

function [idx, how] = auto_window(t, spd, spdtgt)
%AUTO_WINDOW  Longest constant-target stretch where speed has settled,
%   trimming the initial transient and any end-of-run droop.
n = numel(t);
if isempty(spdtgt); spdtgt = zeros(n,1); end
if isempty(spd);    spd = spdtgt; end

% dominant (most common) nonzero target
tv = spdtgt(spdtgt ~= 0);
if isempty(tv); tgt = median(spdtgt); else; tgt = mode(round(tv)); end

% speed_measured has strong per-sample ripple (PLL/aliasing), so test the
% SETTLED condition on a smoothed speed - otherwise ripple spikes fragment
% the window. The reported statistics still use the raw samples.
win = max(5, round(0.05 * n));           % ~5% of the record
spd_s = movmean(spd, win);
onTarget = abs(spdtgt - tgt) < max(1, 0.02*abs(tgt));      % target held
settled  = abs(spd_s - tgt) < max(1, 0.15*abs(tgt));       % smoothed within 15%
good = onTarget & settled;

% longest contiguous run of "good"
d = diff([0; good(:); 0]);
runStart = find(d == 1); runEnd = find(d == -1) - 1;
if isempty(runStart)
    idx = false(n,1); how = 'auto: NO settled window found - pass [t0 t1]'; return;
end
[~, kk] = max(runEnd - runStart);
i0 = runStart(kk); i1 = runEnd(kk);

% trim the first 15% of the run (settling transient)
i0 = i0 + round(0.15 * (i1 - i0));
idx = false(n,1); idx(i0:i1) = true;
how = sprintf('auto: settled @ target=%.4g RPM', tgt);
end

function col = getcol(T, varargin)
%GETCOL  First column of T matching any given name (exact then substring,
%   case-insensitive). [] if none present.
vn = T.Properties.VariableNames; col = [];
for k = 1:numel(varargin)
    idx = find(strcmpi(vn, varargin{k}), 1);
    if isempty(idx); idx = find(contains(vn, varargin{k}, 'IgnoreCase', true), 1); end
    if ~isempty(idx); col = T{:, idx}; return; end
end
end
