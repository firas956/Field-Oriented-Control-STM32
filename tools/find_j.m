[w_rpm, t] = read_foc_bin('coastdown.bin');
w = w_rpm * 2*pi/60;                    % -> rad/s mechanical

m  = t > 0.23 & t < 0.38;               % decay only: after torque removal,
tw = t(m) - t(find(m,1));               %   before the PLL loses lock
ww = movmean(w(m), 201);                % smooth the hall ripple

% (w - w0) = -(B/J)*Int(w dt) - (Tc/J)*t   -> linear least squares
A  = cumtrapz(tw, ww);
dW = ww - ww(1);
th = [A(:), tw(:)] \ dW(:);

B_over_J  = -th(1);      % [1/s]
Tc_over_J = -th(2);      % [rad/s^2]
fprintf('B/J = %.4g 1/s   Tc/J = %.4g rad/s^2\n', B_over_J, Tc_over_J);
% then, with J from the K_t anchor:  B = J*B_over_J;  Tc = J*Tc_over_J;