% =========================================================
% SIMULATION 1: Sonar Equation & Range Calculation
% Sustainable DSL Monitoring System
% All three operating frequencies: 38, 120, 200 kHz
% =========================================================
% HOW TO RUN: Press F5 or click "Run" in MATLAB
% =========================================================

clear; clc; close all;

%% --- SYSTEM PARAMETERS ---

frequencies = [38e3, 120e3, 200e3];        % Hz: fish, krill, zooplankton
freq_labels = {'38 kHz (Fish)', '120 kHz (Krill)', '200 kHz (Zooplankton)'};

% Eco-friendly (proposed) vs Conventional source levels (dB re 1 µPa @ 1m)
SL_eco         = [180, 175, 170];          % Low-power eco system
SL_conventional = [210, 205, 200];         % Typical fishery sonar

% Target Strength (dB) — representative for DSL organisms
TS = [-45, -55, -65];                      % fish, krill, zooplankton

% Detection threshold (minimum received level for detection)
% Deep quiet ocean: DT ~ 0 dB (echo equals noise floor)
DT = 0;                                    % dB

% Noise Level (ambient ocean noise, dB re 1 µPa)
% Deep open ocean below thermocline is very quiet
NL = 30;                                   % dB re 1 µPa (deep quiet ocean)

% Figure of Merit (max allowable two-way transmission loss)
% FOM = SL + TS - NL - DT
% Range achieved when 2*TL = FOM

% Absorption coefficients (dB/km) — increases with frequency
alpha = [10, 34, 60];                      % for 38, 120, 200 kHz (approx)

% Depth range to evaluate
depth = 200:10:1000;                       % meters (DSL range)

%% --- CALCULATE DETECTION RANGE ---

fprintf('=== SONAR RANGE CALCULATION RESULTS ===\n\n');

figure('Name','Sonar Range vs Depth','Color','white','Position',[100 100 1200 500]);

for i = 1:3
    % Figure of Merit
    FOM_eco  = SL_eco(i)  + TS(i) - NL - DT;
    FOM_conv = SL_conventional(i) + TS(i) - NL - DT;

    % Two-way Transmission Loss: TL = 20*log10(R) + alpha*R/1000
    % Solve for range R where 2*TL = FOM
    % We sweep range and find where received level crosses threshold
    R = 1:5:3000;  % range in meters

    % Transmission Loss (spherical spreading + absorption)
    TL = 20*log10(R) + alpha(i) .* R ./ 1000;

    % Received Echo Level
    EL_eco  = SL_eco(i)  + TS(i) - 2*TL;
    EL_conv = SL_conventional(i) + TS(i) - 2*TL;

    % Detection range = max range where EL > NL + DT
    threshold = NL + DT;
    det_range_eco  = max(R(EL_eco  >= threshold));
    det_range_conv = max(R(EL_conv >= threshold));

    fprintf('--- %s ---\n', freq_labels{i});
    fprintf('  Eco-Friendly  Detection Range: %.0f m\n', det_range_eco);
    fprintf('  Conventional  Detection Range: %.0f m\n', det_range_conv);
    fprintf('  Range Sacrifice (eco vs conv): %.0f m (%.1f%%)\n\n', ...
        det_range_conv - det_range_eco, ...
        100*(det_range_conv - det_range_eco)/det_range_conv);

    % Plot Echo Level vs Range
    subplot(1, 3, i);
    plot(R, EL_eco,  'b-',  'LineWidth', 2); hold on;
    plot(R, EL_conv, 'r--', 'LineWidth', 2);
    yline(threshold, 'k:', 'LineWidth', 1.5, 'Label', 'Detection Threshold');
    xline(det_range_eco,  'b:', 'LineWidth', 1.2);
    xline(det_range_conv, 'r:', 'LineWidth', 1.2);
    xlabel('Range (m)'); ylabel('Echo Level (dB re 1 µPa)');
    title(freq_labels{i});
    legend('Eco-Friendly', 'Conventional', 'Location', 'northeast');
    grid on; ylim([-50 100]); xlim([0 3000]);
end

sgtitle('Sonar Echo Level vs Range — Eco-Friendly vs Conventional', 'FontSize', 14, 'FontWeight', 'bold');

%% --- BAR CHART: Detection Range Comparison ---

det_eco  = zeros(1,3);
det_conv = zeros(1,3);

for i = 1:3
    R = 1:5:3000;
    TL = 20*log10(R) + alpha(i) .* R ./ 1000;
    EL_eco  = SL_eco(i)  + TS(i) - 2*TL;
    EL_conv = SL_conventional(i) + TS(i) - 2*TL;
    threshold = NL + DT;
    det_eco(i)  = max(R(EL_eco  >= threshold));
    det_conv(i) = max(R(EL_conv >= threshold));
end

figure('Name','Detection Range Comparison','Color','white','Position',[100 650 700 400]);
bar_data = [det_eco; det_conv]';
b = bar(bar_data, 0.7);
b(1).FaceColor = [0.2 0.5 0.8];  % Blue = eco
b(2).FaceColor = [0.8 0.2 0.2];  % Red  = conventional
set(gca, 'XTickLabel', {'38 kHz', '120 kHz', '200 kHz'});
ylabel('Max Detection Range (m)');
title('Detection Range: Eco-Friendly vs Conventional');
legend('Eco-Friendly (Low Power)', 'Conventional', 'Location', 'northeast');
grid on;

fprintf('=== Simulation 1 Complete. See figures. ===\n');
