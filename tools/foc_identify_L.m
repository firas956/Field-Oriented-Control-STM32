function L = foc_identify_L(binFile, R, varargin)
%FOC_IDENTIFY_L  Estimate stator inductance from a closed-loop current-step .bin.
%
%   L = foc_identify_L(binFile, R)
%   L = foc_identify_L(binFile, R, 'Name',Value, ...)
%
%   Capture: at STANDSTILL, step id_target from 0 to a known value (e.g. 2 A)
%   with iq_target = 0 (pure d-axis current makes no torque, so the rotor
%   stays put -> this identifies L_d ~= L_q on a surface PMSM). Log i_d at
%   20 kHz; the record MUST contain a bit of pre-step baseline and the settled
%   plateau. Extract to a .bin, then run this.
%
%   Why not a plain exponential fit: your current step is CLOSED-LOOP, so
%   i_d(t) follows the loop dynamics, not the open-loop L/R constant. This
%   routine simulates your exact PI loop (matching pid.c) with L as the only
%   free parameter and fits it to the measured rise. It also reports an
%   independent initial-slope estimate as a cross-check.
%
%   Required: R  = stator resistance [Ohm] from the DC test.
%   Options (name-value):
%     'Kp'   d-axis proportional gain      (default 1.5, from motor_control.c)
%     'Ki'   d-axis integral gain          (default 200)
%     'fs'   sample rate [Hz]              (default 20000)
%     'Vmax' PI output clamp [V]           (default 24*0.57735 = 13.86)
%     'Iref' commanded step amplitude [A]  (default: auto from plateau)
%     'FitMs' fit-window length after step (default 30 ms)

if nargin < 2 || isempty(R) || R <= 0
    error('foc_identify_L:R', 'pass the measured stator resistance R (Ohm) as the 2nd argument');
end
p = inputParser;
p.addParameter('Kp',   1.5);
p.addParameter('Ki',   200);
p.addParameter('fs',   20000);
p.addParameter('Vmax', 24*0.57735027);
p.addParameter('Iref', []);
p.addParameter('FitMs', 30);
p.parse(varargin{:});
o = p.Results;

[i_meas, t, fs, ch] = read_foc_bin(binFile, o.fs);
if isempty(i_meas); L = []; return; end
if ~strcmpi(ch, 'id')
    warning('foc_identify_L:chan', ['channel looks like "%s", not id. L needs a d-axis ' ...
            'step at standstill (rotor still). Continuing anyway.'], ch);
end
N  = numel(i_meas);
Ts = 1/fs;

% ---- locate the step (needs pre-step baseline + settled plateau) ----
baseline = median(i_meas(1:round(0.05*N)));
plateau  = median(i_meas(round(0.80*N):end));
if abs(plateau - baseline) < 0.1
    error('foc_identify_L:nostep', ['no clear step found (baseline=%.3f, plateau=%.3f). ' ...
          'The record must contain pre-step baseline and the settled plateau.'], baseline, plateau);
end
thr = baseline + 0.05*(plateau - baseline);
n0  = find(i_meas > thr, 1, 'first');
iref = o.Iref; if isempty(iref); iref = plateau; end   % loop drives i_d -> reference

nFit = min(round(o.FitMs/1000*fs), N - n0);
seg  = n0 : n0 + nFit - 1;
y    = i_meas(seg);

% ---- closed-loop simulation of pid.c, parameterised by L ----
Kin = o.Ki * Ts;                 % = Ki_normalized in pid.c
i0  = i_meas(n0);
integ0 = R * i0;                 % steady voltage that was holding the baseline
    function isim = simulate(L)
        a = exp(-R*Ts/L);        % exact ZOH discretisation of 1/(R+sL)
        isim = zeros(nFit,1);
        i = i0; integ = integ0;
        for k = 1:nFit
            e = iref - i;
            integ = integ + Kin*e;
            integ = min(max(integ, -o.Vmax), o.Vmax);   % anti-windup clamp
            v = min(max(o.Kp*e + integ, -o.Vmax), o.Vmax);
            i = a*i + (v/R)*(1 - a);
            isim(k) = i;
        end
    end

% ---- fit L (search in log space; base MATLAB, no toolbox) ----
cost = @(logL) sum( (simulate(10^logL) - y).^2 );
logL = fminbnd(cost, log10(1e-6), log10(1e-1));
L    = 10^logL;

% ---- independent initial-slope cross-check (R-free) ----
% just after the step i_d~0 and integ~0, so v ~ Kp*iref and v ~ L*di/dt.
ns    = n0 : min(n0 + round(0.0005*fs), N);
pp    = polyfit(t(ns), i_meas(ns), 1);
slope = pp(1);
L_slope = (o.Kp * iref) / slope;

% ---- report ----
tau = L / R;  fbw = o.Kp/(2*pi*L);
fprintf('\n=== L identification (%s) ===\n', binFile);
fprintf('R (given)        : %.4g Ohm\n', R);
fprintf('step             : %.3f -> %.3f A  (iref=%.3f A) at t=%.4f s\n', baseline, plateau, iref, t(n0));
fprintf('L (loop fit)     : %.4g H  = %.3f mH\n', L, L*1e3);
fprintf('L (init. slope)  : %.4g H  = %.3f mH   [cross-check, R-free]\n', L_slope, L_slope*1e3);
fprintf('elec. time const : tau = L/R = %.3f ms\n', 1e3*tau);
fprintf('  (if the two L estimates disagree, suspect v_d saturation, wrong R, or a rotor that moved)\n');
for wbw = 2*pi*[500 800]
    fprintf('suggested gains @ %.0f Hz bw:  Kp = L*wbw = %.3f ,  Ki = R*wbw = %.3f\n', ...
            wbw/(2*pi), L*wbw, R*wbw);
end

% ---- plot measured vs fitted ----
figure('Name',['L fit: ' binFile],'NumberTitle','off','Color','w');
tf = (0:nFit-1)*Ts*1e3;
plot(tf, y, 'b-', 'DisplayName','measured'); hold on;
plot(tf, simulate(L), 'r--', 'LineWidth',1.3, 'DisplayName',sprintf('loop fit, L=%.3f mH',L*1e3));
grid on; xlabel('time after step [ms]'); ylabel('i_d [A]');
title(sprintf('%s  -  R=%.3g \\Omega, K_p=%.3g, K_i=%.3g', ...
      strrep(binFile,'_','\_'), R, o.Kp, o.Ki));
legend('show','Location','southeast');

if nargout == 0; clear L; end
end
