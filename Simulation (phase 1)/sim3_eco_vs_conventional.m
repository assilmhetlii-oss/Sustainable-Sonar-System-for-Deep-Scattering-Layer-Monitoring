% =========================================================
% SIMULATION 3: Eco-Friendly vs Conventional Sonar
%               Acoustic Exposure & Impact Comparison
% =========================================================
% HOW TO RUN: Press F5 or click "Run" in MATLAB
% =========================================================

clear; clc; close all;

%% --- PARAMETERS ---

% Marine mammal safe exposure limit (NOAA, SEL criterion)
% Sound Exposure Level threshold: ~183 dB re 1 µPa²·s (cetaceans)
SEL_threshold = 183;

% Source parameters
SL_eco  = 175;      % dB re 1 µPa @ 1m — eco system (120 kHz)
SL_conv = 205;      % dB re 1 µPa @ 1m — conventional
freq    = 120e3;    % Hz
alpha   = 34;       % dB/km absorption at 120 kHz

% Pulse durations
tau_eco  = 0.001;   % 1 ms — short pulse
tau_conv = 0.010;   % 10 ms — typical conventional pulse

% Duty cycle (fraction of time sonar is transmitting)
duty_eco  = 0.01;   % 1%  — intermittent
duty_conv = 0.30;   % 30% — frequent pinging

% Operation duration (hours)
T_hours = 8;
T_sec   = T_hours * 3600;

% Range from sonar to organism
R = 1:5:2000;   % meters

%% --- SOUND PRESSURE LEVEL vs RANGE ---

TL = 20*log10(R) + alpha .* R ./ 1000;

SPL_eco  = SL_eco  - TL;
SPL_conv = SL_conv - TL;

%% --- SOUND EXPOSURE LEVEL (SEL) vs RANGE ---
% SEL = SPL + 10*log10(tau * N_pings)
% N_pings = duty_cycle * T / tau

N_pings_eco  = duty_eco  * T_sec / tau_eco;
N_pings_conv = duty_conv * T_sec / tau_conv;

SEL_eco  = SPL_eco  + 10*log10(tau_eco  * N_pings_eco);
SEL_conv = SPL_conv + 10*log10(tau_conv * N_pings_conv);

% Safe distance: range at which SEL drops below threshold
safe_R_eco  = min(R(SEL_eco  <= SEL_threshold));
safe_R_conv = min(R(SEL_conv <= SEL_threshold));

fprintf('=== ACOUSTIC EXPOSURE ANALYSIS ===\n');
fprintf('Eco-Friendly  — Safe distance for marine mammals: %.0f m\n', safe_R_eco);
fprintf('Conventional  — Safe distance for marine mammals: %.0f m\n', safe_R_conv);
fprintf('Eco system reduces exclusion zone by: %.0f m (%.1f%%)\n\n', ...
    safe_R_conv - safe_R_eco, 100*(safe_R_conv - safe_R_eco)/safe_R_conv);

%% --- PLOT 1: SPL vs Range ---

figure('Name','SPL vs Range','Color','white','Position',[100 50 800 450]);
plot(R, SPL_eco,  'b-',  'LineWidth', 2.5); hold on;
plot(R, SPL_conv, 'r--', 'LineWidth', 2.5);
yline(SEL_threshold, 'k:', 'LineWidth', 1.5, 'Label','Mammal Safety Threshold');
xline(safe_R_eco,  'b:', 'LineWidth', 1.5);
xline(safe_R_conv, 'r:', 'LineWidth', 1.5);
text(safe_R_eco+20,  160, sprintf('%.0fm', safe_R_eco),  'Color','blue');
text(safe_R_conv+20, 160, sprintf('%.0fm', safe_R_conv), 'Color','red');
xlabel('Range from Sonar (m)'); ylabel('Sound Pressure Level (dB re 1 µPa)');
title('Sound Pressure Level vs Range — 120 kHz');
legend('Eco-Friendly', 'Conventional', 'Location','northeast');
grid on; xlim([0 2000]); ylim([80 220]);

%% --- PLOT 2: SEL vs Range ---

figure('Name','SEL vs Range','Color','white','Position',[950 50 800 450]);
plot(R, SEL_eco,  'b-',  'LineWidth', 2.5); hold on;
plot(R, SEL_conv, 'r--', 'LineWidth', 2.5);
yline(SEL_threshold, 'k:', 'LineWidth', 2, 'Label','NOAA Safety Limit (183 dB SEL)');
xlabel('Range from Sonar (m)'); ylabel('Sound Exposure Level SEL (dB re 1 µPa²·s)');
title(sprintf('Cumulative Acoustic Exposure over %d Hours', T_hours));
legend('Eco-Friendly', 'Conventional', 'Location','northeast');
grid on; xlim([0 2000]);

%% --- PLOT 3: Parameter Comparison Bar Chart ---

figure('Name','System Comparison','Color','white','Position',[100 600 900 400]);

params = {'Source Level (dB)', 'Pulse Duration (ms×10)', 'Duty Cycle (%)', ...
          'Safe Zone (m÷10)', 'N Pings per Hour (÷1000)'};

vals_eco  = [SL_eco, tau_eco*1000*10,  duty_eco*100,  safe_R_eco/10,   N_pings_eco/T_hours/1000];
vals_conv = [SL_conv, tau_conv*1000*10, duty_conv*100, safe_R_conv/10,  N_pings_conv/T_hours/1000];

x = 1:length(params);
bar_w = 0.35;
b1 = bar(x - bar_w/2, vals_eco,  bar_w, 'FaceColor', [0.2 0.5 0.8]);
hold on;
b2 = bar(x + bar_w/2, vals_conv, bar_w, 'FaceColor', [0.8 0.2 0.2]);
set(gca, 'XTick', x, 'XTickLabel', params, 'FontSize', 9);
ylabel('Value (scaled — see labels)');
title('System Parameter Comparison: Eco-Friendly vs Conventional');
legend('Eco-Friendly', 'Conventional', 'Location', 'northwest');
grid on;

fprintf('=== Simulation 3 Complete. See figures. ===\n');
