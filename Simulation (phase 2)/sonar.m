N = 400;              % number of pings
ranges = 45:0.05:55;  % displayed range

sigma = 0.12;         % sonar resolution

stationary = zeros(size(ranges));
moving = zeros(size(ranges));

for k = 1:N

    %% Stationary
    r = 50;

    stationary = stationary + exp(-(ranges-r).^2/(2*sigma^2));

    %% Oscillating
    z = 500 + 2*sin(2*pi*k/N);

    r = 550 - z;

    moving = moving + exp(-(ranges-r).^2/(2*sigma^2));

end

stationary = stationary/max(stationary);
moving = moving/max(moving);

figure

subplot(2,1,1)
plot(ranges,stationary,'LineWidth',2)
xlabel('Detected range (m)')
ylabel('Normalized echo')
title('Stationary ROV')
grid on

subplot(2,1,2)
plot(ranges,moving,'LineWidth',2)
xlabel('Detected range (m)')
ylabel('Normalized echo')
title('ROV Oscillating \pm2 m')
grid on