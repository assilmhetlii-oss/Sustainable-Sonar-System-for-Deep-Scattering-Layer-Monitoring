% =========================================================
% SIMULATION 4: Biomass Estimation from Acoustic Backscatter
%               Multi-frequency classification & density
% =========================================================
% HOW TO RUN: Press F5 or click "Run" in MATLAB
% =========================================================

clear; clc; close all;

%% --- PARAMETERS ---

depth = 0:2:1000;           % meters
freq_labels = {'38 kHz (Fish)', '120 kHz (Krill)', '200 kHz (Zooplankton)'};

% DSL layer depth and thickness
DSL_depth = 550;
DSL_width = 80;

% Peak Volume Backscatter Sv (dB re m^-1) per frequency
Sv_peak_dB = [-55, -60, -68];

% Background Sv
Sv_bg = -90;

% Conversion factor: backscatter to biomass density
% Biomass (kg/m^3) = 10^(Sv/10) / sigma_bs_per_unit_mass
% We use empirical approximate factors per organism type
% (simplified from CCAMLR and fisheries acoustics literature)
biomass_factor = [2500, 1200, 600];  % kg/m^3 per unit linear Sv

%% --- GENERATE Sv PROFILES ---

Sv_dB   = zeros(3, length(depth));
Sv_lin  = zeros(3, length(depth));

for i = 1:3
    Sv_dB(i,:)  = Sv_bg + (Sv_peak_dB(i) - Sv_bg) .* ...
                  exp(-0.5 .* ((depth - DSL_depth) ./ DSL_width).^2);
    Sv_lin(i,:) = 10.^(Sv_dB(i,:) ./ 10);   % Convert dB to linear
end

%% --- BIOMASS ESTIMATION ---

% Biomass density (kg/m^3) at each depth bin
biomass_density = zeros(3, length(depth));
for i = 1:3
    biomass_density(i,:) = Sv_lin(i,:) .* biomass_factor(i);
end

% Total integrated biomass per m^2 (kg/m^2)
dz = depth(2) - depth(1);   % depth bin size = 2m
integrated_biomass = sum(biomass_density, 2) .* dz;

fprintf('=== BIOMASS ESTIMATION RESULTS ===\n');
for i = 1:3
    fprintf('%s → Integrated Biomass: %.4f kg/m²\n', freq_labels{i}, integrated_biomass(i));
end

total_biomass = sum(integrated_biomass);
fprintf('\nTotal (all organisms): %.4f kg/m²\n', total_biomass);

% Area estimation (assume monitoring cell of 10km × 10km)
area_m2 = 10000 * 10000;
total_biomass_tons = total_biomass * area_m2 / 1000;
fprintf('Estimated biomass in 10km×10km monitoring cell: %.1f metric tons\n\n', total_biomass_tons);

%% --- PLOT 1: Sv Profiles ---

figure('Name','Sv Profiles','Color','white','Position',[100 50 500 600]);
colors = {'b', 'g', 'r'};
for i = 1:3
    plot(Sv_dB(i,:), depth, '-', 'Color', colors{i}, 'LineWidth', 2); hold on;
end
set(gca, 'YDir', 'reverse');
xlabel('Volume Backscatter Sv (dB re 1 m^{-1})');
ylabel('Depth (m)');
title('Multi-Frequency Sv Profiles');
legend(freq_labels, 'Location', 'southeast');
grid on; ylim([0 1000]);

%% --- PLOT 2: Biomass Density vs Depth ---

figure('Name','Biomass vs Depth','Color','white','Position',[650 50 500 600]);
for i = 1:3
    plot(biomass_density(i,:)*1e6, depth, '-', 'Color', colors{i}, 'LineWidth', 2); hold on;
end
set(gca, 'YDir', 'reverse');
xlabel('Biomass Density (×10^{-6} kg/m^3)');
ylabel('Depth (m)');
title('Estimated Biomass Density vs Depth');
legend(freq_labels, 'Location', 'southeast');
grid on; ylim([0 1000]);

%% --- PLOT 3: Frequency Differencing for Classification ---
% Frequency differencing: dB(120kHz) - dB(38kHz) → distinguishes krill from fish

diff_120_38  = Sv_dB(2,:) - Sv_dB(1,:);   % krill indicator
diff_200_120 = Sv_dB(3,:) - Sv_dB(2,:);   % zooplankton indicator

figure('Name','Frequency Differencing','Color','white','Position',[100 700 900 380]);
subplot(1,2,1);
plot(diff_120_38, depth, 'm-', 'LineWidth', 2);
set(gca, 'YDir', 'reverse');
xlabel('ΔSv 120–38 kHz (dB)'); ylabel('Depth (m)');
title('Krill Indicator (120–38 kHz)');
grid on; ylim([0 1000]);
xline(-5, 'k--', 'Label', 'Krill Threshold');

subplot(1,2,2);
plot(diff_200_120, depth, 'c-', 'LineWidth', 2);
set(gca, 'YDir', 'reverse');
xlabel('ΔSv 200–120 kHz (dB)'); ylabel('Depth (m)');
title('Zooplankton Indicator (200–120 kHz)');
grid on; ylim([0 1000]);
xline(-5, 'k--', 'Label', 'Zoo Threshold');

sgtitle('Frequency Differencing — Organism Classification', 'FontSize', 13, 'FontWeight', 'bold');

%% --- PLOT 4: Integrated Biomass Bar ---

figure('Name','Integrated Biomass','Color','white','Position',[100 400 500 380]);
bar_vals = integrated_biomass * 1e4;   % scale for readability
b = bar(bar_vals, 0.6);
b.FaceColor = 'flat';
b.CData = [0.2 0.4 0.8; 0.2 0.7 0.3; 0.8 0.3 0.2];
set(gca, 'XTickLabel', {'Fish (38kHz)', 'Krill (120kHz)', 'Zoo (200kHz)'});
ylabel('Integrated Biomass (×10^{-4} kg/m^2)');
title('Biomass Estimate per Organism Group');
grid on;

fprintf('=== Simulation 4 Complete. See figures. ===\n');
