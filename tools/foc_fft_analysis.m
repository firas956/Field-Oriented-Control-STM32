function S = foc_fft_analysis(csvFile, window, polePairs)
%FOC_FFT_ANALYSIS  Harmonic (FFT) analysis of the FOC phase / dq currents.
%
%   S = foc_fft_analysis()                          - picker, auto window, p=2
%   S = foc_fft_analysis(csvFile)
%   S = foc_fft_analysis(csvFile,[t0 t1])           - explicit window [s]
%   S = foc_fft_analysis(csvFile,[t0 t1],polePairs)
%
%   WHY: a true PMSM has a sinusoidal back-EMF, so under FOC its phase
%   current is nearly a pure fundamental (low THD, tiny 5th/7th). A
%   trapezoidal (BLDC-style) machine driven by sinusoidal FOC shows strong
%   5th & 7th harmonics in the phase current, which appear as a 6*f_e
%   ripple in i_d / i_q. The size of those harmonics is the discriminator,
%   and it doubles as a torque-ripple figure of merit for controller tuning.
%
%   WHAT it does: resamples the (irregularly sampled) log onto a uniform
%   grid, removes DC, applies a Hann window, FFTs i_a and i_d/i_q, locks to
%   the fundamental, and reports harmonic amplitudes, THD, and the 6*f_e
%   content - with an explicit Nyquist-adequacy check.
%
%   !! SAMPLING CAVEAT !! The definitive PMSM-vs-BLDC answer needs the 5th
%   and 7th harmonics resolved, i.e. sample rate > ~15*f_e. The telemetry
%   logger (~300-500 Hz) cannot do this above a few hundred RPM - the tool
%   WARNS when the harmonics of interest exceed Nyquist and the verdict is
%   then "inconclusive". For a clean answer, feed this an ISR-rate (20 kHz)
%   capture, or better, run the open-circuit back-EMF spin-down test.

if nargin < 1 || isempty(csvFile)
    [f, p] = uigetfile({'*.csv','CSV log (*.csv)'}, 'Select FOC log file');
    if isequal(f, 0); S = []; return; end
    csvFile = fullfile(p, f);
end
if nargin < 2; window = []; end
if nargin < 3 || isempty(polePairs); polePairs = 2; end

% ---- 20 kHz single-channel binary capture: dedicated path ----
if endsWith(lower(csvFile), '.bin')
    S = fft_bin(csvFile, window, polePairs);   % arg2 reused as mean RPM
    return;
end

% ---- load ----
T = foc_load(csvFile);
t = getcol(T, 'time'); if isempty(t); error('foc_fft:noTime','no time column'); end
t = t - t(1);
spd = getcol(T, 'speed_measured');
ia  = getcol(T, 'i_abc.a', 'i_abc_a');
idq_d = getcol(T, 'i_dq.d', 'i_dq_d');
idq_q = getcol(T, 'i_dq.q', 'i_dq_q');
if isempty(ia) && isempty(idq_q); error('foc_fft:noCurrent','no current columns'); end

% ---- window ----
if ~isempty(window)
    m = t >= window(1) & t <= window(2); how = sprintf('[%.3f %.3f]s', window(1), window(2));
else
    [m, how] = auto_window(t, spd);
end
tw = t(m);
if numel(tw) < 64; error('foc_fft:short','window too short (%d samples)', numel(tw)); end

% ---- uniform resample (log timestamps are not evenly spaced) ----
dt = median(diff(tw));  fs = 1/dt;
tu = (tw(1):dt:tw(end)).';
N  = numel(tu);
mean_speed = mean(spd(m), 'omitnan');
fe = abs(mean_speed)/60 * polePairs;         % expected electrical fundamental [Hz]

fprintf('\n=== FOC current FFT ===\n');
fprintf('file  : %s\n', csvFile);
fprintf('window: %s  N=%d  fs=%.1f Hz  Nyquist=%.1f Hz  df=%.2f Hz\n', how, N, fs, fs/2, fs/N);
fprintf('speed : %.0f RPM  ->  f_e=%.1f Hz (p=%d)\n', mean_speed, fe, polePairs);

% ---- Nyquist adequacy: need up to 7*fe to judge the machine ----
adequate = (7*fe) < 0.9*(fs/2);
if ~adequate
    fprintf(2, ['\n!! NYQUIST WARNING: 7*f_e=%.0f Hz exceeds Nyquist=%.0f Hz.\n' ...
                '   The 5th/7th harmonics are aliased; PMSM-vs-BLDC verdict is\n' ...
                '   INCONCLUSIVE from this log. Use a >~%.0f Hz capture.\n'], ...
                7*fe, fs/2, 15*fe);
end

S = struct('file',csvFile,'fs',fs,'N',N,'fe',fe,'polePairs',polePairs, ...
           'nyquist_adequate',adequate);

% ---- spectra ----
fig = figure('Name',['FFT: ' csvFile],'NumberTitle','off','Color','w');
tl = tiledlayout(fig, 2, 1, 'TileSpacing','compact','Padding','compact');
[~, fn] = fileparts(csvFile);
title(tl, {strrep(fn,'_','\_'), sprintf('f_e=%.1f Hz, fs=%.0f Hz%s', fe, fs, ...
      tern(adequate,'',' — HARMONICS ALIASED'))});

% phase-current spectrum: fundamental + THD (the PMSM/BLDC discriminator)
if ~isempty(ia)
    [f, A] = single_sided(interp1(tw, ia(m), tu, 'linear'), fs);
    [S.fund_Hz, S.fund_A, kfund] = find_fundamental(f, A, fe);
    [S.harm, S.THD] = harmonics(f, A, S.fund_Hz, 9, fs);
    ax = nexttile(tl);
    plot(ax, f, A, 'b'); hold(ax,'on');
    mark_harmonics(ax, S.fund_Hz, 9, fs, max(A));
    xline(ax, fs/2, 'k:', 'Nyquist');
    xlim(ax, [0, fs/2]); grid(ax,'on');
    xlabel(ax,'Hz'); ylabel(ax,'|i_a| [A]');
    title(ax, sprintf('Phase current spectrum  —  THD=%.1f%%', 100*S.THD));
    legend(ax, 'show', 'Location','best');
    fprintf('\nphase current i_a:  fundamental %.1f Hz = %.4f A\n', S.fund_Hz, S.fund_A);
    fprintf('  harmonic   freq(true) [alias]   amp[A]   %%offund\n');
    for h = 2:numel(S.harm)+1
        hs = S.harm(h-1);
        fprintf('  %2dth      %6.0f  [%6.0f]    %.4f   %5.1f%%\n', ...
                h, hs.f_true, hs.f_obs, hs.amp, 100*hs.rel);
    end
    fprintf('  THD (2..9) = %.1f %%\n', 100*S.THD);
end

% dq spectrum: 6*fe is the compact non-sinusoidal signature
if ~isempty(idq_q)
    [fq, Aq] = single_sided(interp1(tw, idq_q(m), tu, 'linear'), fs);
    ax = nexttile(tl); plot(ax, fq, Aq, 'r', 'DisplayName','i_q'); hold(ax,'on');
    if ~isempty(idq_d)
        [~, Ad] = single_sided(interp1(tw, idq_d(m), tu, 'linear'), fs);
        plot(ax, fq, Ad, 'Color',[0 0.6 0], 'DisplayName','i_d');
    end
    for kk = [6 12]
        fo = alias(kk*fe, fs);
        xline(ax, fo, '--', sprintf('%d f_e', kk), 'Color',[0.5 0.5 0.5]);
        S.(sprintf('dq_%dfe_A', kk)) = peak_near(fq, Aq, fo, 3*fs/N);
    end
    xline(ax, fs/2, 'k:', 'Nyquist');
    xlim(ax, [0, fs/2]); grid(ax,'on');
    xlabel(ax,'Hz'); ylabel(ax,'|i_{dq}| [A]');
    title(ax, 'dq-frame spectrum (fundamental -> DC; look for 6 f_e)');
    legend(ax,'show','Location','best');
    fprintf('\ndq frame:  6*f_e content = %.4f A', S.dq_6fe_A);
    if isfield(S,'dq_12fe_A'); fprintf('   12*f_e = %.4f A', S.dq_12fe_A); end
    fprintf('  (aliased)\n');
end

% ---- verdict ----
fprintf('\n--- verdict ---\n');
if ~adequate
    fprintf(['Sample rate too low: 5th/7th harmonics alias, so the machine\n' ...
             'type CANNOT be decided from this log. What IS reliable here:\n' ...
             ' - the fundamental at %.1f Hz confirms p=%d (%.0f RPM).\n' ...
             'Next: capture i_a at the 20 kHz ISR rate, or run the open-\n' ...
             'circuit back-EMF spin-down test, then re-run this tool.\n'], ...
             S.fund_Hz, polePairs, mean_speed);
elseif isfield(S,'THD')
    if S.THD < 0.05
        fprintf('THD=%.1f%%, 5th/7th small -> SINUSOIDAL back-EMF => PMSM.\n', 100*S.THD);
    elseif S.THD < 0.10
        fprintf('THD=%.1f%% -> mildly non-sinusoidal PMSM (some cogging/saliency).\n', 100*S.THD);
    else
        fprintf('THD=%.1f%%, strong 5th/7th -> TRAPEZOIDAL back-EMF => BLDC-type.\n', 100*S.THD);
    end
end
end

% ======================================================================
function [f, A] = single_sided(x, fs)
x = x(:); x = x - mean(x, 'omitnan'); x(isnan(x)) = 0;
N = numel(x);
w = hann(N); x = x .* w;
X = fft(x);
A = abs(X(1:floor(N/2)+1)) / sum(w) * 2;   % Hann-corrected single-sided amp
f = (0:floor(N/2)).' * fs / N;
end

function [ff, fa, k] = find_fundamental(f, A, fe_hint)
% largest peak within +/-40% of the predicted fe (falls back to global max)
band = f > 0.6*fe_hint & f < 1.4*fe_hint;
if any(band)
    idx = find(band); [~, j] = max(A(idx)); k = idx(j);
else
    [~, k] = max(A(2:end)); k = k + 1;
end
ff = f(k); fa = A(k);
end

function [H, thd] = harmonics(f, A, f1, nmax, fs)
df = f(2) - f(1); H = struct('f_true',{},'f_obs',{},'amp',{},'rel',{});
A1 = peak_near(f, A, f1, 2*df); sumsq = 0;
for h = 2:nmax
    ft = h*f1; fo = alias(ft, fs);
    a = peak_near(f, A, fo, 2*df);
    H(h-1) = struct('f_true',ft,'f_obs',fo,'amp',a,'rel',a/A1);
    sumsq = sumsq + a^2;
end
thd = sqrt(sumsq) / A1;
end

function a = peak_near(f, A, f0, tol)
if f0 <= 0 || f0 >= f(end); a = 0; return; end
sel = abs(f - f0) <= tol;
if any(sel); a = max(A(sel)); else; a = 0; end
end

function fo = alias(ftrue, fs)
% observed frequency of ftrue after sampling at fs (folding)
fo = mod(ftrue, fs); if fo > fs/2; fo = fs - fo; end
end

function mark_harmonics(ax, f1, nmax, fs, ymax)
for h = 1:nmax
    fo = alias(h*f1, fs);
    if fo <= 0 || fo >= fs/2; continue; end
    c = tern(h==1, 'k', [0.6 0.6 0.6]);
    xline(ax, fo, '--', sprintf('%d', h), 'Color', c, 'HandleVisibility','off');
end
end

function [m, how] = auto_window(t, spd)
n = numel(t);
if isempty(spd); m = true(n,1); how = 'full log'; return; end
win = max(5, round(0.05*n)); ss = movmean(spd, win);
tgt = median(ss(ss > 0.5*max(abs(ss))));
good = abs(ss - tgt) < 0.15*max(1, abs(tgt));
d = diff([0; good(:); 0]); r0 = find(d==1); r1 = find(d==-1)-1;
if isempty(r0); m = true(n,1); how = 'full log (no settle found)'; return; end
[~, k] = max(r1 - r0); i0 = r0(k) + round(0.15*(r1(k)-r0(k)));
m = false(n,1); m(i0:r1(k)) = true;
how = sprintf('auto @ %.0f RPM', tgt);
end

function y = tern(c, a, b); if c; y = a; else; y = b; end; end

function col = getcol(T, varargin)
vn = T.Properties.VariableNames; col = [];
for k = 1:numel(varargin)
    idx = find(strcmpi(vn, varargin{k}), 1);
    if isempty(idx); idx = find(contains(vn, varargin{k}, 'IgnoreCase', true), 1); end
    if ~isempty(idx); col = T{:, idx}; return; end
end
end

% ======================================================================
function S = fft_bin(binFile, rpmArg, polePairs)
%FFT_BIN  Spectrum of a single-channel 20 kHz capture (see read_foc_bin).
%   Uniform sampling -> no resampling needed. f_e comes from the signal
%   itself for a speed capture, else from the RPM passed as arg 2, else
%   parsed from a *_2000rpm.bin filename. Reuses single_sided/alias/peak_near.
[x, t, fs, ch] = read_foc_bin(binFile);
N = numel(x);

meanRPM = NaN;
if strcmp(ch, 'speed'); meanRPM = mean(x); end
if is_scalar_num(rpmArg); meanRPM = rpmArg; end          % explicit arg wins
if isnan(meanRPM); meanRPM = rpm_from_name(binFile); end
fe = abs(meanRPM)/60 * polePairs;

[f, A] = single_sided(x, fs);
units  = tern(strcmp(ch,'speed'), 'RPM', 'A');

fprintf('\n=== FOC %s FFT (20 kHz capture) ===\n', ch);
fprintf('file : %s\n', binFile);
fprintf('N=%d  fs=%.0f Hz  Nyquist=%.0f Hz  df=%.3f Hz  T=%.3f s\n', N, fs, fs/2, fs/N, t(end));
if ~isnan(fe)
    fprintf('speed: %.0f RPM -> f_e=%.1f Hz (p=%d)\n', meanRPM, fe, polePairs);
else
    fprintf('speed unknown: pass RPM as 2nd arg or name file *_2000rpm.bin - no harmonic markers\n');
end

S = struct('file',binFile,'channel',ch,'fs',fs,'N',N,'fe',fe,'polePairs',polePairs);

fig = figure('Name',['FFT: ' binFile],'NumberTitle','off','Color','w');
tl  = tiledlayout(fig,2,1,'TileSpacing','compact','Padding','compact');
[~,fn] = fileparts(binFile); title(tl, strrep(fn,'_','\_'));

ax = nexttile(tl); plot(ax,f,A,'b'); hold(ax,'on');        % full 0..Nyquist
if ~isnan(fe)
    for kk = [1 2 6 12]
        fo = alias(kk*fe, fs);
        if fo>0 && fo<fs/2; xline(ax,fo,'--',sprintf('%d f_e',kk),'Color',[.5 .5 .5]); end
        S.(sprintf('h%dfe_amp',kk)) = peak_near(f, A, alias(kk*fe,fs), 3*fs/N);
    end
end
xline(ax,fs/2,'k:','Nyquist'); xlim(ax,[0 fs/2]); grid(ax,'on');
xlabel(ax,'Hz'); ylabel(ax,sprintf('|%s| [%s]',ch,units)); title(ax,'Full spectrum');

ax2 = nexttile(tl); plot(ax2,f,A,'b'); hold(ax2,'on');     % low-frequency zoom
if ~isnan(fe)
    for kk = [1 2 6 12]
        fo = alias(kk*fe, fs);
        if fo>0; xline(ax2,fo,'--',sprintf('%d f_e',kk),'Color',[.5 .5 .5]); end
    end
    fmax = min(fs/2, max(20*fe, 500));
else
    fmax = min(fs/2, 1000);
end
xlim(ax2,[0 fmax]); grid(ax2,'on');
xlabel(ax2,'Hz'); ylabel(ax2,sprintf('|%s| [%s]',ch,units)); title(ax2,sprintf('Zoom 0-%.0f Hz',fmax));

if ~isnan(fe)
    fprintf('\n6*f_e content = %.4g %s (torque/speed ripple at 6th electrical harmonic)\n', ...
            peak_near(f,A,alias(6*fe,fs),3*fs/N), units);
end
end

function tf = is_scalar_num(v)
tf = isnumeric(v) && isscalar(v) && ~isnan(v);
end

function rpm = rpm_from_name(f)
[~,name] = fileparts(lower(f));
tok = regexp(name, '(\d+)\s*rpm', 'tokens', 'once');
if isempty(tok); rpm = NaN; else; rpm = str2double(tok{1}); end
end
