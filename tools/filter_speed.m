function filter_speed(csvfile)
% FILTER_SPEED  Offline filtering of the logged FOC speed measurement.
%
%   filter_speed                 -> uses logged_data.csv next to this script
%   filter_speed('my_log.csv')   -> explicit file
%
% Reads the logger CSV (semicolon-separated, decimal commas), extracts
% foc_core.speed_measured and compares three filters:
%
%   1. causal 1st-order LPF   - exactly what the firmware would run,
%                               including its lag. Use this to pick fc.
%   2. zero-phase 1st-order   - same cutoff, filtered forward+backward.
%                               No lag, offline analysis reference only.
%   3. moving median + mean   - spike-tolerant baseline for comparison.
%
% Prints steady-state statistics and plots time series + spectrum.
% No toolboxes required (base MATLAB only).

%% ---------------- config ----------------------------------------------
if nargin < 1
    csvfile = fullfile(fileparts(mfilename('fullpath')), 'logged_data.csv');
end
fc         = 25;     % LPF cutoff [Hz] - candidate value for the firmware
med_win_ms = 40;     % moving median/mean window [ms]
pole_pairs = 2;      % motor pole pairs (hall ripple frequency marker)
t_ss       = NaN;    % steady-state window start [s]; NaN = auto-detect

%% ---------------- load ------------------------------------------------
txt   = strrep(fileread(csvfile), ',', '.');       % decimal comma -> dot
lines = strsplit(strtrim(txt), {'\r\n','\n'});
hdr   = strtrim(strsplit(lines{1}, ';'));
hdr   = hdr(~cellfun(@isempty, hdr));
ncol  = numel(hdr);

L = string(lines(2:end));
L = L(strlength(L) > 0 & count(L, ';') == count(string(lines{1}), ';'));
vals = str2double(split(L(:), ';'));               % rows x (ncol + trailing)
vals = vals(:, 1:ncol);

ix  = @(name) find(strcmp(hdr, name), 1);
t   = vals(:, ix('time'));
sp  = vals(:, ix('foc_core.speed_measured'));
tgt = vals(:, ix('foc_core.speed_target'));

%% ---------------- resample to uniform grid ----------------------------
% Log timestamps are not perfectly uniform (3-5 ms); IIR filtering assumes
% a fixed dt, so interpolate onto the median sample period.
dt  = median(diff(t));
fs  = 1/dt;
tu  = (t(1):dt:t(end))';
sp  = interp1(t, sp,  tu, 'linear');
tgu = interp1(t, tgt, tu, 'linear');

%% ---------------- steady-state window ---------------------------------
% settled = the longest contiguous run where the ~200 ms smoothed speed
% stays within +/-10 % of the *instantaneous* target. Tracking the target
% sample-by-sample keeps ramps, direction changes and the stop at the end
% of the log out of the window.
if isnan(t_ss)
    m  = movmean(sp, max(3, round(0.2*fs)));
    ok = abs(tgu) > 0 & abs(m - tgu) < 0.10*abs(tgu);
    d  = diff([0; ok(:); 0]);
    rs = find(d == 1);  re = find(d == -1) - 1;
    if isempty(rs)
        ss = round(0.4*numel(sp)):numel(sp);          % fallback: last 60 %
    else
        [~, k] = max(re - rs);
        i0 = min(rs(k) + round(0.2*fs), re(k));       % drop 0.2 s settle tail
        ss = i0:re(k);
    end
else
    ss = find(tu >= t_ss, 1):numel(sp);
end

%% ---------------- filters ---------------------------------------------
% 1st-order IIR: y[n] = (1-a)*y[n-1] + a*x[n], a from the continuous pole
a  = 1 - exp(-2*pi*fc*dt);
b1 = a;  a1 = [1, a-1];

% (1) causal - firmware preview (initial state = first sample, no startup dip)
y_c = filter(b1, a1, sp, (1-a)*sp(1));

% (2) zero-phase - forward + backward pass with edge padding
pad = round(fs/fc);
xp  = [repmat(sp(1),pad,1); sp; repmat(sp(end),pad,1)];
y1  = filter(b1, a1, xp, (1-a)*xp(1));
y2  = flipud(filter(b1, a1, flipud(y1), (1-a)*y1(end)));
y_z = y2(pad+1:end-pad);

% (3) moving median (kills spikes) then moving mean (smooths the rest)
w   = max(3, round(med_win_ms/1000*fs));
y_m = movmean(movmedian(sp, w), w);

%% ---------------- statistics ------------------------------------------
names = {'raw', sprintf('causal LPF %g Hz',fc), ...
         sprintf('zero-phase %g Hz',fc), sprintf('med+mean %g ms',med_win_ms)};
sigs  = {sp, y_c, y_z, y_m};

fprintf('\nFile: %s\n', csvfile);
fprintf('fs = %.1f Hz  |  steady-state: %.3f - %.3f s  (%d samples, %.2f s)\n', ...
        fs, tu(ss(1)), tu(ss(end)), numel(ss), numel(ss)*dt);

f_hall  = mean(sp(ss))/60 * pole_pairs * 6;              % 6 edges / elec rev
f_alias = abs(f_hall - fs*round(f_hall/fs));             % folded into [0,fs/2]
fprintf('hall ripple 6*f_e = %.0f Hz -> appears aliased at %.0f Hz (fs/2 = %.0f Hz)\n\n', ...
        f_hall, f_alias, fs/2);

fprintf('%-22s %10s %10s %10s %12s\n', 'signal','mean','std','p2p','std vs raw');
s_raw = std(sp(ss));
for k = 1:numel(sigs)
    x = sigs{k}(ss);
    fprintf('%-22s %10.1f %10.1f %10.1f %11.2fx\n', ...
            names{k}, mean(x), std(x), max(x)-min(x), std(x)/s_raw);
end
fprintf(['\nNote: the causal trace is what the MCU filter will produce ' ...
         '(group delay ~ %.0f ms);\nthe zero-phase trace is the no-lag ' ...
         'reference for offline analysis only.\n\n'], 1000/(2*pi*fc));

%% ---------------- plots: time domain ----------------------------------
% top: all speeds overlaid.  below: one axes per signal, x/y linked so
% zooming any of them zooms all.
figure('Name','FOC speed - time','Color','w');
clr = [0.70 0.70 0.70;   % raw
       0.85 0.33 0.10;   % causal LPF
       0.93 0.69 0.13;   % zero-phase
       0.00 0.45 0.74];  % med+mean
ax  = gobjects(1, numel(sigs)+1);

ax(1) = subplot(3,2,[1 2]); hold on; grid on;
for k = 1:numel(sigs)
    plot(tu, sigs{k}, 'Color', clr(k,:), 'DisplayName', names{k});
end
plot(tu, tgu, 'k--', 'DisplayName', 'target');
plot(tu(ss([1 1])),     ylim, 'k:', 'HandleVisibility', 'off');  % ss window
plot(tu(ss([end end])), ylim, 'k:', 'HandleVisibility', 'off');
ylabel('speed [RPM]'); title('all signals together');
legend('Location','best');

for k = 1:numel(sigs)
    ax(k+1) = subplot(3,2,2+k); hold on; grid on;
    plot(tu, sigs{k}, 'Color', clr(k,:));
    plot(tu, tgu, 'k--');
    ylabel('RPM');
    title(sprintf('%s   (ss std = %.1f RPM)', names{k}, std(sigs{k}(ss))));
end
xlabel(ax(4), 'time [s]'); xlabel(ax(5), 'time [s]');
linkaxes(ax, 'xy'); xlim(ax(1), [tu(1) tu(end)]);

%% ---------------- plots: spectrum -------------------------------------
% steady-state amplitude spectrum of the RAW speed only, drawn as a line
% on a log amplitude scale. (The DFT is discrete - one bin every fs/N Hz
% - the line just connects the bins for readability.) DC bin removed.
figure('Name','FOC speed - spectrum','Color','w'); hold on; grid on;
N    = numel(ss);
f    = (0:N-1)'*fs/N;
keep = f > 0 & f <= fs/2;
x    = sp(ss) - mean(sp(ss));
A    = 2*abs(fft(x))/N;
plot(f(keep), A(keep), 'Color', [0.45 0.45 0.45], ...
     'DisplayName', 'raw speed');
set(gca, 'YScale', 'log');
% aliased hall-ripple harmonics: n*6*f_e folded into [0, fs/2]
f_e = mean(sp(ss))/60 * pole_pairs;
for nh = 1:4
    fa = abs(nh*6*f_e - fs*round(nh*6*f_e/fs));
    h  = plot(fa*[1 1], ylim, 'r:', 'HandleVisibility', 'off');
    if nh == 1
        set(h, 'HandleVisibility', 'on', ...
               'DisplayName', 'hall ripple harmonics (aliased)');
    end
end
xlabel('frequency [Hz]'); ylabel('amplitude [RPM]');
title('steady-state spectrum of raw speed (single-sided DFT)');
legend('Location','best');
end
