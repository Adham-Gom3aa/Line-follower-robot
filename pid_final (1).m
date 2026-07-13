%% LINE FOLLOWER PID DESIGN USING IDENTIFIED G(s)

clc; close all;
s = tf('s');

% ------------------------------------------------------------------------
% 1) Continuous-time plant from identification
% ------------------------------------------------------------------------
% Derive the plant transfer function G(s) using system identification or a research paper then reference it.
disp('Loaded Plant Transfer Function G(s):');
% Original continuous-time transfer function
num = 0.3581;
den = [1 4.657 2.845];

Gs = tf(num, den);
scale = 1/0.3581;   % or any factor to normalize output
Gs = Gs * scale;
% Sampling time for the digital controller
Ts = 0.01;

%% -----------------------------------------------------------------------
% 2) Discretization of the Plant
%% -----------------------------------------------------------------------
% Convert G(s) to G(z) using two different discretization methods.
% Zero-Order Hold (ZOH)
Gz_zoh = c2d(Gs, Ts, 'zoh');
disp('Transfer Function with Zero-Order Hold (ZOH):');
disp(Gz_zoh);

% Tustin (Bilinear)
Gz_Tustin = c2d(Gs, Ts, 'tustin');
disp('Transfer Function with Tustin Approximation:');
disp(Gz_Tustin);

%% -----------------------------------------------------------------------
% 3) PID Controller 
%% -----------------------------------------------------------------------
% Tune, Save & Extract the PID controller parameters using MATLAB PID Tuner.
disp('Discrete-Time PID Controller C(z):');
%pidTuner(Gz_zoh)
disp(C);

% Compute PID controller transfer function in Z-domain
% Extract PID gains
Kp = C.Kp;
Ki = C.Ki;
Kd = C.Kd;

disp(['PID Gains: Kp=', num2str(Kp), ', Ki=', num2str(Ki), ', Kd=', num2str(Kd)]);

% Compute PID controller transfer function in S-domain.
% Continuous-time PID transfer function
C_s = pid(Kp, Ki, Kd);
disp('Continuous-Time PID C(s):'); disp(C_s);

%% -----------------------------------------------------------------------
% 4) Closed-Loop Transfer Functions
%% -----------------------------------------------------------------------

% Compute the closed-loop transfer function in Z-domain.
% Discrete-time closed-loop
GoL = Gz_zoh * C;
Tz = feedback(GoL, 1);
disp('Closed-loop Discrete Transfer Function T(z):');
disp(Tz);

% Compute the closed-loop transfer function in S-domain.
% Continuous-time closed-loop
GoL_s = Gs * C_s;
T_s = feedback(GoL_s, 1);
disp('Closed-loop Continuous Transfer Function T(s):');
disp(T_s);

%% -----------------------------------------------------------------------
% 5) Step Responses
%% -----------------------------------------------------------------------

% Plot the step response before and after PID in the Z-domain
% Step response before PID
figure;
step(Gz_zoh,0.2);
title('Step Response Before PID');

% Step response after PID
figure;
step(Tz);
title('Step Response After PID');

%% -----------------------------------------------------------------------
% 6) Steady-State Error (SSE)

%% -----------------------------------------------------------------------
% Compute steady-state error (SSE) before applying PID
[y_before, t_b] = step(Gz_zoh);  % before PID
ess_before = 1 - y_before(end);
disp(['SSE BEFORE PID: ', num2str(ess_before)]);

% Compute steady-state error (SSE) after applying PID
[y_after_z, t_a] = step(Tz);  % after PID
ess_after = 1 - y_after_z(end);
disp(['SSE AFTER PID: ', num2str(ess_after)]);


%% -----------------------------------------------------------------------
% 7) Transient Response Characteristics
%% -----------------------------------------------------------------------

% Obtain transient response values (rise time, settling time, overshoot %, peak time, delay time) before PID.
% Before PID
S_before = stepinfo(Gz_zoh);
disp('Transient Response BEFORE PID:');
disp(S_before);

% Obtain transient response values (rise time, settling time, overshoot %, peak time, delay time) after PID.
% After PID
S_after = stepinfo(Tz);
disp('Transient Response AFTER PID:');
disp(S_after);

%% -----------------------------------------------------------------------
% 8) Plot the error signal of the system
%% -----------------------------------------------------------------------
%% Plot error signal before PID
r_before = ones(size(y_before));       % reference input (unit step)
e_before = r_before - y_before;        % error signal

figure;
plot(t_b, e_before, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Error e(t)');
title('Error Signal BEFORE PID');

%% Plot error signal after PID
r_after = ones(size(y_after_z));       % reference input (unit step)
e_after = r_after - y_after_z;         % error signal

figure;
plot(t_a, e_after, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Error e(t)');
title('Error Signal AFTER PID');

%% -----------------------------------------------------------------------
% 9) Analyze the effect of the Zero-Order Hold (ZOH) before and after PID tuning.
%% -----------------------------------------------------------------------

figure;
step(Tz); hold on;
step(Gz_zoh);
title('ZOH + PID vs ZOH');
legend('ZOH+PID','ZOH');

%% -----------------------------------------------------------------------
% 10) Perform stability analysis in Z-domain using isstable()
%% -----------------------------------------------------------------------

% Check stability of open-loop system
if isstable(GoL)
    disp('Open-loop system is STABLE.');
else
    disp('Open-loop system is UNSTABLE.');
end

% Check stability of closed-loop system
if isstable(Tz)
    disp('Closed-loop system is STABLE.');
else
    disp('Closed-loop system is UNSTABLE.');
end

%% -----------------------------------------------------------------------
% 11) Plot poles and zeros of the system in the Z-plane
%% -----------------------------------------------------------------------

% Poles and zeros of the closed-loop system
figure;
pzmap(Tz);
title('Poles and Zeros of Closed-Loop System in Z-plane');

% Poles and zeros of the open-loop system (GoL)
figure;
pzmap(GoL);
title('Poles and Zeros of Open-Loop System in Z-plane');

%% -----------------------------------------------------------------------
% 12) Plot root locus for the open-loop system and closed-loop system.
%% -----------------------------------------------------------------------

figure;
rlocus(GoL);
grid on;
title('Root Locus of Open-Loop System (Z-domain)');

figure;
rlocus(Tz);
grid on;
title('Root Locus of Closed-Loop System (Z-domain)');
