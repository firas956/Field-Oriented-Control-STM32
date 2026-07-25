function estimate_tau(csvFile, loLevel, hiLevel)
% ESTIMATE_TAU  Characterise the closed-loop speed response over a full step passage.
%
%   estimate_tau                       % speed.csv, whole 1400 -> 2100 passage
%   estimate_tau('speed.csv')          % same, explicit file
%   estimate_tau('speed.csv',700,1400) % a different step level
%
% Analyses the ENTIRE time the target sits at hiLevel: from the step-up
% (loLevel -> hiLevel) until it drops back down. The measured speed
% overshoots, so a second-order + dead-time model is fitted:
%
%   y(t) = yf - (yf-y0)*exp(-z*wn*x)*( cos(wd*x) + z/sqrt(1-z^2)*sin(wd*x) )
%          with x = t - t0 - L,  wd = wn*sqrt(1-z^2),  for t > t0+L
%
% and it reports the damping z, natural frequency wn, the envelope time
% constant tau = 1/(z*wn), the dead time L, overshoot and 2% settling time.
% Uses fminsearch only, so no toolboxes are required.
% CSV is the CubeMonitor export (';' delimiter, ',' decimal separator).

if nargin < 1 || isempty(csvFile), csvFile = 'speed_pi0.025.csv'; end
if nargin < 2 || isempty(loLevel), loLevel = 1400; end
if nargin < 3 || isempty(hiLevel), hiLevel = 2100; end

% ---------------------------------------------- read + fix decimal comma
raw = strrep(fileread(csvFile), ',', '.');
tmp = [tempname '.csv']; fid = fopen(tmp,'w'); fwrite(fid,raw); fclose(fid);
c = onCleanup(@() delete(tmp)); %#ok<NASGU>
T = readtable(tmp, detectImportOptions(tmp,'Delimiter',';'));
t   = T{:,1};    % time [s]
sm  = T{:,14};   % foc_core.speed_measured
tgt = T{:,15};   % foc_core.speed_target

% ------------------------------ locate the full passage (up-step -> down-step)
mid = (loLevel + hiLevel)/2;
iUp = find(tgt(1:end-1) < mid & tgt(2:end) >= hiLevel-1, 1);
if isempty(iUp), error('No %g -> %g step found in %s', loLevel, hiLevel, csvFile); end
iDn = find(tgt((iUp+1):end) < hiLevel-1, 1) + iUp;      % first drop back down
if isempty(iDn), iDn = numel(t); end
t0 = t(iUp);  tEnd = t(iDn);

w  = t >= t0-0.02 & t < tEnd;      % whole passage
tt = t(w) - t0;  yy = sm(w);
y0 = mean(sm(t > t0-0.02 & t < t0));            % baseline
yf0 = mean(sm(t > tEnd-0.5 & t < tEnd));        % final value from plateau tail
step = yf0 - y0;

% ------------------------------------- second-order + dead-time model & fit
model = @(p) sos_step(tt, y0, p(1), p(2), p(3), p(4));   % p=[yf L z wn]
cost  = @(p) sse(p, tt, yy, y0, step);
p0    = [yf0, 0.010, 0.65, 50];
opt   = optimset('MaxFunEvals',5e4,'MaxIter',5e4,'TolX',1e-6,'TolFun',1e-4);
p     = fminsearch(cost, p0, opt);
yf = p(1); L = p(2); z = p(3); wn = p(4);
tau = 1/(z*wn);
rmse = sqrt(mean((model(p)-yy).^2));

% ------------------------------------- overshoot & 2% settling from the fit
tf   = linspace(0, tt(end), 20000)';
yfit = sos_step(tf, y0, yf, L, z, wn);
Mp   = (max(yfit)-yf)/(yf-y0)*100;
band = 0.02*abs(yf-y0);
ks   = find(abs(yfit-yf) > band, 1, 'last');
ts   = tf(ks);

fprintf('Full %g -> %g passage: %.3f s .. %.3f s  (%.0f ms wide)\n', ...
        loLevel, hiLevel, t0, tEnd, (tEnd-t0)*1e3);
fprintf('  measured step   : %.0f -> %.0f rpm\n', y0, yf);
fprintf('  dead time L     : %.1f ms\n', L*1e3);
fprintf('  damping  zeta   : %.3f\n', z);
fprintf('  nat. freq wn    : %.1f rad/s  (%.1f Hz)\n', wn, wn/2/pi);
fprintf('  envelope tau    : %.1f ms   (= 1/(zeta*wn))\n', tau*1e3);
fprintf('  overshoot       : %.1f %%\n', Mp);
fprintf('  2%% settling time: %.0f ms\n', ts*1e3);
fprintf('  fit rmse        : %.1f rpm over whole passage\n', rmse);

% ------------------------------------------------------------------- plot
figure('Color','w','Name','Full step passage: 2nd-order fit');
plot(tt*1e3, yy, '.', 'Color',[.6 .6 .6], 'MarkerSize',4); hold on;
plot(tf*1e3, yfit, 'k-', 'LineWidth',2);
plot(tt*1e3, tgt(w), 'r--', 'LineWidth',1.5);
yline(yf, 'b:');
grid on; box on;
xlabel('time after step [ms]'); ylabel('speed [rpm]');
legend('measured (raw)','2nd-order + dead-time fit','target','Location','southeast');
title(sprintf('1400\\rightarrow2100 passage  |  \\zeta=%.2f, \\omega_n=%.0f rad/s, \\tau_{env}=%.0f ms', ...
      z, wn, tau*1e3));
end

% ========================================================================
function y = sos_step(tt, y0, yf, L, z, wn)
% Underdamped second-order step response with dead time L.
z = min(max(z,0.02),0.999);
y = y0*ones(size(tt));
k = tt > L;  x = tt(k) - L;
wd = wn*sqrt(1-z^2);
env = exp(-z*wn*x);
y(k) = yf - (yf-y0).*env.*(cos(wd*x) + (z/sqrt(1-z^2))*sin(wd*x));
end

function e = sse(p, tt, yy, y0, step)
% Penalised sum of squared errors for fminsearch.
yf = p(1); L = p(2); z = p(3); wn = p(4);
if L < 0 || z <= 0.02 || z >= 0.999 || wn <= 0 || abs(yf-y0) < 0.2*abs(step)
    e = 1e18; return;                 % keep the search in a sane region
end
e = sum((sos_step(tt, y0, yf, L, z, wn) - yy).^2);
end
