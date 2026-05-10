% =========================================================
% SIMULATION 2: Volume Backscatter (Sv) vs Depth
%               Deep Scattering Layer (DSL) Detection
% =========================================================
% HOW TO RUN: Press F5 or click "Run" in MATLAB
% =========================================================

clear; clc; close all;

%% --- PARAMETERS ---

depth = 0:1:1000;           % Depth in meters
frequencies = [38e3, 120e3, 200e3];
freq_labels  = {'38 kHz (Fish)', '120 kHz (Krill)', '200 kHz (Zooplankton)'};
colors       = {'b', 'g', 'r'};

% DSL characteristics (depth of peak scattering layer)
DSL_center  = 550;          % meters — typical daytime DSL depth
DSL_width   = 80;           % meters — thickness of layer
DSL_night   = 80;           % meters — DSL migrates up at night

% Sv baseline (background noise level, dB)
Sv_background = -90;

% Peak Sv values per frequency (organisms scatter differently)
% Fish with swim bladders → strongest at 38 kHz
% Krill → strongest at 120 kHz
% Zooplankton → strongest at 200 kHz
Sv_peak = [-55, -60, -68];  % dB re 1 m^-1

%% --- MODEL: Sv PROFILE vs DEPTH ---

% DSL modeled as a Gaussian scattering peak
% Sv(z) = Sv_peak * exp(-0.5 * ((z - z_DSL) / width)^2)

Sv_day   = zeros(3, length(depth));
Sv_night = zeros(3, length(depth));

for i = 1:3
    % Daytime: DSL at 550m
    Sv_day(i,:)   = Sv_background + (Sv_peak(i) - Sv_background) .* ...
                    exp(-0.5 .* ((depth - DSL_center) ./ DSL_width).^2);

    % Nighttime: DSL migrates up to ~80m
    Sv_night(i,:) = Sv_background + (Sv_peak(i) - Sv_background) .* ...
                    exp(-0.5 .* ((depth - DSL_night) ./ DSL_width).^2);
end

%% --- PLOT 1: Sv vs Depth (all frequencies, day vs night) ---

figure('Name','Backscatter vs Depth','Color','white','Position',[100 50 1300 600]);

for i = 1:3
    subplot(1,3,i);
    plot(Sv_day(i,:),   depth, '-',  'Color', colors{i}, 'LineWidth', 2); hold on;
    plot(Sv_night(i,:), depth, '--', 'Color', colors{i}, 'LineWidth', 2);
    set(gca, 'YDir', 'reverse');   % Depth increases downward
    xlabel('Volume Backscatter Sv (dB re 1 m^{-1})');
    ylabel('Depth (m)');
    title(freq_labels{i});
    legend('Daytime DSL', 'Nighttime (Migrated)', 'Location', 'southeast');
    grid on;
    ylim([0 1000]);
    xlim([-100 -40]);
end

sgtitle('Volume Backscatter (Sv) vs Depth — DSL Diel Migration', 'FontSize', 14, 'FontWeight', 'bold');

%% --- PLOT 2: DSL Detection Echogram (2D: time × depth) ---

% Simulate a 24-hour echogram
time_hours = 0:0.5:24;       % hours
Sv_echogram = zeros(length(depth), length(time_hours));

for t = 1:length(time_hours)
    hr = time_hours(t);

    % DSL center depth: sinks at dawn (~6h), rises at dusk (~18h)
    if hr >= 6 && hr <= 18
        z_center = 550;  % Daytime: deep
    elseif hr < 6
        z_center = 80 + (550 - 80) * (hr / 6);   % Rising pre-dawn
    else
        z_center = 550 - (550 - 80) * ((hr - 18) / 6);  % Rising post-dusk
    end

    % Use 120 kHz (krill) for echogram
    Sv_echogram(:,t) = Sv_background + (Sv_peak(2) - Sv_background) .* ...
                       exp(-0.5 .* ((depth - z_center) ./ DSL_width).^2);
end

figure('Name','DSL Echogram','Color','white','Position',[100 700 1000 400]);
imagesc(time_hours, depth, Sv_echogram);
set(gca, 'YDir', 'reverse');
colormap(jet); colorbar;
caxis([-95 -58]);
xlabel('Time (hours)'); ylabel('Depth (m)');
title('Simulated 24-Hour Echogram — 120 kHz | DSL Diel Migration', 'FontSize', 13);
hold on;
xline(6,  'w--', 'LineWidth', 1.5); text(6.1, 50, 'Dawn', 'Color','white', 'FontSize', 9);
xline(18, 'w--', 'LineWidth', 1.5); text(18.1, 50, 'Dusk', 'Color','white', 'FontSize', 9);

%% --- DETECT DSL LAYER ---

fprintf('=== DSL LAYER DETECTION ===\n');
threshold_detect = -80;   % dB — anything above this = DSL detected

for i = 1:3
    dsl_zone = depth(Sv_day(i,:) > threshold_detect);
    if ~isempty(dsl_zone)
        fprintf('%s → DSL detected from %.0f m to %.0f m depth\n', ...
            freq_labels{i}, min(dsl_zone), max(dsl_zone));
    else
        fprintf('%s → DSL not detected at this threshold\n', freq_labels{i});
    end
end

fprintf('\n=== Simulation 2 Complete. See figures. ===\n');
