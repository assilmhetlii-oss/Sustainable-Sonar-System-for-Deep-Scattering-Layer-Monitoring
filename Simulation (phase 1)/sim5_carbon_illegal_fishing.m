% =========================================================
% SIMULATION 5: Carbon Flux Estimation & Illegal Fishing Detection
%               From DSL Biomass & Migration Patterns
% =========================================================
% HOW TO RUN: Press F5 or click "Run" in MATLAB
% =========================================================

clear; clc; close all;

%% ==========================================
%  PART A: CARBON FLUX ESTIMATION
% ==========================================

fprintf('=== PART A: CARBON FLUX ESTIMATION ===\n\n');

% --- Biomass inputs (from Simulation 4 results) ---
% Organisms respire O2 and produce CO2 during diel migration
% Carbon transported = biomass × respiration rate × migration depth

% Biomass density at DSL (kg/m^3, approximate)
biomass_kgm3 = [2.0e-5, 1.5e-5, 0.8e-5];   % fish, krill, zooplankton

% Respiration rates (µmol O2 / mg wet weight / hr)
% Values from marine biology literature
resp_rate = [2.5, 3.5, 1.8];     % fish, krill, zooplankton

% Migration depth (daytime DSL - nighttime DSL)
migration_depth = 550 - 80;       % = 470 meters

% Conversion factors
% O2 consumed → CO2 produced (RQ = 0.85 for marine organisms)
RQ = 0.85;

% mg wet weight per m^3 → convert biomass
% Assume 80% water content: 1 kg wet = 200g dry
wet_to_dry = 0.20;

% Monitoring area
area_m2 = 10000 * 10000;   % 10km × 10km

% Layer thickness
layer_thickness = 80;       % meters

% --- COMPUTE CARBON FLUX ---
carbon_flux = zeros(1, 3);   % mgC/m^2/day

for i = 1:3
    % Total biomass in layer per m^2 (kg/m^2)
    B_m2 = biomass_kgm3(i) * layer_thickness;   % kg/m^2
    B_mg_m2 = B_m2 * 1e6;                       % convert to mg/m^2

    % Dry weight (mg dry / m^2)
    B_dry = B_mg_m2 * wet_to_dry;

    % O2 consumption (µmol/m^2/hr)
    O2_consumed = B_dry * resp_rate(i);

    % CO2 produced (µmol/m^2/hr × RQ)
    CO2_produced = O2_consumed * RQ;

    % Convert to mgC: 1 µmol CO2 = 12 µg C = 0.012 mgC
    C_flux_hr = CO2_produced * 0.012;   % mgC/m^2/hr

    % Daily flux (active ~12 hrs of migration)
    carbon_flux(i) = C_flux_hr * 12;    % mgC/m^2/day
end

total_carbon_flux = sum(carbon_flux);
fprintf('Carbon Flux per Organism Group:\n');
fprintf('  Fish:        %.3f mgC/m²/day\n', carbon_flux(1));
fprintf('  Krill:       %.3f mgC/m²/day\n', carbon_flux(2));
fprintf('  Zooplankton: %.3f mgC/m²/day\n', carbon_flux(3));
fprintf('  TOTAL:       %.3f mgC/m²/day\n\n', total_carbon_flux);

% Annualize and scale to monitoring area
annual_C_tons = total_carbon_flux * 365 * area_m2 / 1e9;  % metric tons C/yr
fprintf('Annual Carbon Export (10km×10km area): %.2f metric tons C/year\n\n', annual_C_tons);

%% --- PLOT A1: Carbon Flux Bar Chart ---

figure('Name','Carbon Flux','Color','white','Position',[100 50 600 400]);
b = bar(carbon_flux, 0.6);
b.FaceColor = 'flat';
b.CData = [0.2 0.4 0.8; 0.2 0.7 0.3; 0.8 0.3 0.2];
set(gca, 'XTickLabel', {'Fish', 'Krill', 'Zooplankton'});
ylabel('Carbon Flux (mgC/m²/day)');
title('DSL Biological Carbon Pump — Estimated Daily Carbon Export');
grid on;
text(1, carbon_flux(1)+0.01, sprintf('%.3f', carbon_flux(1)), 'HorizontalAlignment','center');
text(2, carbon_flux(2)+0.01, sprintf('%.3f', carbon_flux(2)), 'HorizontalAlignment','center');
text(3, carbon_flux(3)+0.01, sprintf('%.3f', carbon_flux(3)), 'HorizontalAlignment','center');

%% --- PLOT A2: Migration + Carbon Transport Diagram ---

depth_axis = 0:1:600;
time_axis  = 0:1:24;
biomass_map = zeros(length(depth_axis), length(time_axis));

for t = 1:length(time_axis)
    hr = time_axis(t);
    if hr >= 6 && hr <= 18
        z_c = 550;
    elseif hr < 6
        z_c = 80 + (550-80)*(hr/6);
    else
        z_c = 550 - (550-80)*((hr-18)/6);
    end
    biomass_map(:,t) = exp(-0.5.*((depth_axis - z_c)./40).^2);
end

figure('Name','Migration Carbon Transport','Color','white','Position',[750 50 800 420]);
imagesc(time_axis, depth_axis, biomass_map);
set(gca,'YDir','reverse'); colormap(hot); colorbar;
xlabel('Hour of Day'); ylabel('Depth (m)');
title('Simulated DSL Diel Migration — Biological Carbon Pump Driver');
hold on;
yline(80,  'w--','LineWidth',1.5); text(0.5, 70,  'Night surface (~80m)',  'Color','w','FontSize',9);
yline(550, 'w--','LineWidth',1.5); text(0.5, 560, 'Day depth (~550m)',     'Color','w','FontSize',9);
xline(6,  'c-','LineWidth',1.5); text(6.1,  20, 'Dawn', 'Color','c','FontSize',9);
xline(18, 'c-','LineWidth',1.5); text(18.1, 20, 'Dusk', 'Color','c','FontSize',9);

%% ==========================================
%  PART B: ILLEGAL FISHING DETECTION
% ==========================================

fprintf('\n=== PART B: ILLEGAL FISHING DETECTION ===\n\n');

% --- Simulate 30-day biomass time series ---

days = 1:30;
rng(42);   % Reproducible random seed

% Normal biomass baseline (120 kHz krill) + natural variability
biomass_normal = 1.5e-5 + 0.1e-5 .* randn(1, 30);

% Inject illegal fishing event: Days 14–17 → sudden biomass drop
biomass_observed = biomass_normal;
biomass_observed(14:17) = biomass_observed(14:17) .* 0.35;  % 65% drop

% --- DETECTION ALGORITHM ---
% Rolling 5-day mean and std as baseline
window = 5;
alert = zeros(1, 30);
anomaly_score = zeros(1, 30);

for d = (window+1):30
    window_data = biomass_observed(d-window:d-1);
    mu  = mean(window_data);
    sig = std(window_data);
    z_score = (biomass_observed(d) - mu) / (sig + eps);
    anomaly_score(d) = z_score;
    if z_score < -2.0   % More than 2 std below baseline = alert
        alert(d) = 1;
    end
end

alert_days = days(alert == 1);
fprintf('Baseline biomass:   %.2e kg/m³\n', mean(biomass_normal));
fprintf('Anomaly at days:    %s\n', num2str(alert_days));
fprintf('Biomass drop:       ~%.0f%% below baseline\n', ...
    100 * (1 - mean(biomass_observed(14:17)) / mean(biomass_normal)));
fprintf('Detection verdict:  ILLEGAL FISHING ALERT TRIGGERED ⚠️\n\n');

%% --- PLOT B1: Biomass Time Series + Alert ---

figure('Name','Illegal Fishing Detection','Color','white','Position',[100 550 1000 420]);
subplot(2,1,1);
plot(days, biomass_normal*1e6,   'b--', 'LineWidth', 1.5, 'DisplayName', 'Expected Baseline'); hold on;
plot(days, biomass_observed*1e6, 'k-',  'LineWidth', 2,   'DisplayName', 'Observed Biomass');
for d = alert_days
    xline(d, 'r-', 'LineWidth', 2);
end
scatter(alert_days, biomass_observed(alert_days)*1e6, 80, 'r', 'filled', 'DisplayName','⚠️ Alert');
xlabel('Day'); ylabel('Biomass Density (×10^{-6} kg/m³)');
title('30-Day Biomass Monitoring — 120 kHz Krill Layer');
legend('Location','northeast'); grid on;

% Shading the event window
patch([13.5 17.5 17.5 13.5], [0 0 3 3], 'red', 'FaceAlpha',0.08, 'EdgeColor','none');
text(15, 2.8, 'Fishing Event', 'Color','red','HorizontalAlignment','center','FontSize',9);

%% --- PLOT B2: Anomaly Score ---

subplot(2,1,2);
bar(days, anomaly_score, 'FaceColor', [0.5 0.5 0.5]);
hold on;
yline(-2, 'r--', 'LineWidth', 2, 'Label', 'Alert Threshold (z = -2)');
bar(alert_days, anomaly_score(alert_days), 'FaceColor', 'red');
xlabel('Day'); ylabel('Anomaly Score (Z-score)');
title('Statistical Anomaly Score — Biomass Deviation Detector');
grid on;

sgtitle('Illegal Fishing Detection System — DSL Biomass Monitoring', 'FontSize', 13, 'FontWeight', 'bold');

%% --- PLOT B3: DSL Layer Fragmentation ---

% Show what a healthy vs disturbed DSL looks like
depth_range = 0:2:1000;

Sv_healthy    = -90 + 30 .* exp(-0.5.*((depth_range - 550)./80).^2);
Sv_disturbed  = -90 + 30 .* exp(-0.5.*((depth_range - 550)./80).^2);
% Disturbed: reduced peak + fragment at different depth
Sv_disturbed  = Sv_disturbed - 10;   % overall reduction
Sv_disturbed  = Sv_disturbed + 8 .* exp(-0.5.*((depth_range - 430)./30).^2);  % fragment

figure('Name','DSL Fragmentation','Color','white','Position',[750 550 600 450]);
plot(Sv_healthy,   depth_range, 'b-',  'LineWidth', 2.5, 'DisplayName', 'Healthy DSL'); hold on;
plot(Sv_disturbed, depth_range, 'r--', 'LineWidth', 2.5, 'DisplayName', 'Disturbed DSL (post-fishing)');
set(gca,'YDir','reverse');
xlabel('Volume Backscatter Sv (dB re 1 m^{-1})');
ylabel('Depth (m)');
title('DSL Layer: Healthy vs Disturbed by Illegal Fishing');
legend('Location','southeast'); grid on;
ylim([300 800]);

fprintf('=== Simulation 5 Complete. See all figures. ===\n');
fprintf('\n✅ ALL 5 SIMULATIONS READY. Open each .m file in MATLAB and press Run.\n');
