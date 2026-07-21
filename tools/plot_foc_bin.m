function [x, t] = plot_foc_bin(binFile, fs, varargin)
%PLOT_FOC_BIN  Time-domain plot of a single-channel .bin capture.
%
%   plot_foc_bin()                        % file picker, fs = 20 kHz
%   plot_foc_bin(binFile)
%   plot_foc_bin(binFile, fs)
%   plot_foc_bin(binFile, fs, 'Name',Value, ...)
%   [x, t] = plot_foc_bin(...)            % also return the samples + time
%
%   Plots one channel logged by the firmware datalogger (speed, i_d OR i_q)
%   and extracted with:
%       (gdb) dump binary memory iq_2000rpm.bin &datalog_buf[0] &datalog_buf[28672]
%   The figure shows the time series with its mean and +/-1sigma / +/-2sigma
%   bands, a stats box (mean/std/RMS/pk-pk), and a value histogram. Channel
%   and units (A for i_d/i_q, RPM for speed) are inferred from the filename,
%   or forced with the 'Channel' option.
%
%   Name-value options:
%     'Window'   [t0 t1]   restrict to a time window in seconds (default: all)
%     'Detrend'  logical   subtract the mean before plotting (default false)
%     'Decimate' k         plot every k-th sample for long records (default 1)
%     'Channel'  char      'speed' | 'id' | 'iq' to override filename guess

if nargin < 1; binFile = ''; end
if nargin < 2 || isempty(fs); fs = 20000; end

p = inputParser;
p.addParameter('Window',   [],   @(v) isempty(v) || (isnumeric(v) && numel(v)==2));
p.addParameter('Detrend',  false,@(v) islogical(v) || ismember(v,[0 1]));
p.addParameter('Decimate', 1,    @(v) isnumeric(v) && isscalar(v) && v>=1);
p.addParameter('Channel',  '',   @(v) ischar(v) || isstring(v));
p.parse(varargin{:});
opt = p.Results;

[x, t, fs, ch] = read_foc_bin(binFile, fs);
if isempty(x); return; end               % picker cancelled
if ~isempty(opt.Channel); ch = char(opt.Channel); end

switch lower(ch)
    case 'speed'; units = 'RPM'; lab = 'speed_{meas}';
    case 'id';    units = 'A';   lab = 'i_d';
    case 'iq';    units = 'A';   lab = 'i_q';
    otherwise;    units = '';    lab = 'signal';
end

% ---- optional time window ----
if ~isempty(opt.Window)
    m = t >= opt.Window(1) & t <= opt.Window(2);
    if ~any(m); error('plot_foc_bin:window', 'window [%g %g] s selects no samples', ...
                      opt.Window(1), opt.Window(2)); end
    t = t(m); x = x(m);
end

% ---- stats on the (windowed) data, before detrend/decimate ----
mu = mean(x); sd = std(x); rms_ = sqrt(mean(x.^2)); pk2pk = max(x) - min(x);

if opt.Detrend; x = x - mu; mu_plot = 0; else; mu_plot = mu; end

% ---- optional display decimation ----
k = round(opt.Decimate);
if k > 1; td = t(1:k:end); xd = x(1:k:end); else; td = t; xd = x; end

% ---- figure ----
fig = figure('Name', sprintf('plot_foc_bin: %s', binFile), 'NumberTitle','off','Color','w');
tl = tiledlayout(fig, 1, 4, 'TileSpacing','compact','Padding','compact');
[~, fn] = fileparts(binFile);
title(tl, strrep(fn,'_','\_'));

% time series (spans 3/4 width)
ax = nexttile(tl, [1 3]); hold(ax, 'on');
fill(ax, [td; flipud(td)], [mu_plot-2*sd + 0*td; flipud(mu_plot+2*sd + 0*td)], ...
     [0.90 0.90 0.96], 'EdgeColor','none', 'DisplayName','\pm2\sigma');
fill(ax, [td; flipud(td)], [mu_plot-sd + 0*td;  flipud(mu_plot+sd + 0*td)], ...
     [0.75 0.80 0.95], 'EdgeColor','none', 'DisplayName','\pm1\sigma');
plot(ax, td, xd, 'b-', 'DisplayName', lab);
yline(ax, mu_plot, 'k-', 'LineWidth', 1.2, 'DisplayName','mean');
grid(ax,'on'); xlabel(ax,'Time [s]'); ylabel(ax, sprintf('%s [%s]', lab, units));
title(ax, sprintf('N=%d   fs=%.0f Hz   T=%.3f s', numel(x), fs, numel(x)/fs));
legend(ax, 'show', 'Location','best'); xlim(ax, [td(1) td(end)]);

txt = sprintf('mean = %.4g %s\nstd  = %.4g %s\nRMS  = %.4g %s\npk-pk= %.4g %s', ...
              mu, units, sd, units, rms_, units, pk2pk, units);
text(ax, 0.99, 0.02, txt, 'Units','normalized', 'HorizontalAlignment','right', ...
     'VerticalAlignment','bottom', 'BackgroundColor',[1 1 1], 'EdgeColor',[0.7 0.7 0.7]);

% value histogram (remaining 1/4 width), sharing the value axis
ax2 = nexttile(tl);
histogram(ax2, x, 40, 'Orientation','horizontal', 'FaceColor',[0.3 0.5 0.9], 'EdgeColor','none');
grid(ax2,'on'); title(ax2,'distribution'); xlabel(ax2,'count');
ylabel(ax2, sprintf('%s [%s]', lab, units));

if nargout == 0; clear x t; end
end
