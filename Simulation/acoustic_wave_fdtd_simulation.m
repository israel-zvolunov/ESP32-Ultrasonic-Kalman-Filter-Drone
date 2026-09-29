clear; clc; close all;

%% 1. physical setup & discretization
c = 343;                % speed of sound
f0 = 40000;             % sonar frequency
L_start = 4;            % start distance
wall_velocity = -1.125; % drone speed
dx = 1 / 2000;    % grid size
dt = 1 * dx / c;        % time step for stable fdtd
x = 0:dx:L_start;       % distance array
Nx = length(x);         

%% 2. source: pulse train generation
cycles = 8;             % pulses count
pulse_duration = cycles * (1/f0); 
sigma = pulse_duration / 6; % pulse variance
t_local = -pulse_duration/2 : dt : pulse_duration/2;
% base sonar ping
single_pulse = 10 * sin(2*pi*f0*t_local) .* exp(-(t_local.^2)/(2*sigma^2));
rng(43);
num_pulses = 5;     
PRI = 1 / 30;           % 30hz rate
train_duration = PRI * num_pulses;         
sim_duration = train_duration + (2.5 * L_start/c); 
t = 0:dt:sim_duration;  
Nt = length(t);

% stack pings over time
source_full = zeros(1, Nt);
pulse_start_times = zeros(1, num_pulses); 
for k = 0 : num_pulses - 1
    t_fire = k * PRI;
    pulse_start_times(k+1) = t_fire;
    idx_start = round(t_fire / dt) + 1;
    idx_end = idx_start + length(single_pulse) - 1;
    if idx_end <= Nt
        source_full(idx_start : idx_end) = single_pulse; 
    end
end

%% 3. fdtd initialization
u = zeros(1, Nx); u_prev = zeros(1, Nx); u_next = zeros(1, Nx); 
noise_level = 0.1;        % awgn noise
sensor_threshold = 0.6;   % hardware comparator level
detections = [];          
last_echo_time = -1;      
refractory_period = 0.002; % blind window to stop double triggering
pulse_counter = 1;        
sensor_readings = zeros(1, Nt); 
geometric_loss = 0.99972;  % signal decay

min_wall_idx = 50;        

%% 4. main fdtd simulation loop

figure('Name', 'Noisy Simulation');
subplot(2,1,1);
hLine = plot(x, u, 'LineWidth', 1.5, 'Color', 'b'); hold on;
hWall = xline(L_start, 'r', 'LineWidth', 4); 
hText = text(L_start, 0.8, ' WALL', 'Color', 'r');
axis([0 L_start -4 4]); grid on;
xlabel('Distance [m]'); ylabel('Pressure');
title('1. Physical Wave Propagation. PRESS ANY KEY TO START...');

subplot(2,1,2);
hSensor = plot(t*1000, sensor_readings, 'k'); hold on;
yline(sensor_threshold, 'r--', 'Threshold');
yline(-sensor_threshold, 'r--', 'Threshold');
axis([0 sim_duration*1000 -2 2]); grid on;
title('2. What the Sensor Actually Sees (Signal + Noise)');
xlabel('Time [ms]'); ylabel('Voltage [V]');

subplot(2,1,1); waitforbuttonpress; 
title('1. Physical Wave Propagation (Acoustic Pressure)');


for n = 1:Nt
    % uav movement tracking
    current_wall_pos = L_start + wall_velocity * t(n);
    wall_idx = round(current_wall_pos / dx);
    if wall_idx < min_wall_idx; wall_idx = min_wall_idx; end
    if wall_idx > Nx; wall_idx = Nx; end
    
    % main fdtd wave wave equation updater
    u_next(2:end-1) = (u(3:end) + u(1:end-2) - u_prev(2:end-1)) * geometric_loss;
    
    % source boundary setup
    if n <= length(source_full) && source_full(n) ~= 0
        u_next(1) = source_full(n);
    else
        u_next(1) = u(2); 
    end
    
    % wall bounce boundary reflection modeling
    R = 0.8; 
    u_next(wall_idx) = (1+R)*u(wall_idx-1) - R*u_prev(wall_idx);
    if wall_idx < Nx; u_next(wall_idx+1 : end) = 0; end 
    
    u_prev = u; u = u_next;
    
    % disable reading while pinging
    is_transmitting = any(abs(t(n) - pulse_start_times) < 0.001);
    if is_transmitting
        measured_val = 0; 
    else
        measured_val = u(1) + noise_level * randn(); 
    end
    sensor_readings(n) = measured_val;
    
    % check if signal cuts threshold level
    if pulse_counter <= num_pulses
        current_tx = pulse_start_times(pulse_counter);
        if ~is_transmitting && (t(n) > last_echo_time + refractory_period) && (t(n) > current_tx)
            if abs(measured_val) > sensor_threshold 
                last_echo_time = t(n);
                tof = t(n) - current_tx; 
                d_true = L_start + wall_velocity * t(n); 
                detections = [detections; tof, t(n), (c*tof)/2, d_true];
                
                subplot(2,1,2); 
                plot(t(n)*1000, measured_val, 'ro', 'MarkerFaceColor', 'r');
                
                pulse_counter = pulse_counter + 1;
            end
        end
    end
    
    % frame updates
    if mod(n, 100) == 0
        set(hLine, 'YData', u);
        set(hWall, 'Value', current_wall_pos);
        set(hText, 'Position', [current_wall_pos, 0.8]);
        set(hSensor, 'XData', t(1:n)*1000, 'YData', sensor_readings(1:n));
        drawnow; 
    end

    if mod(n, round(Nt/10)) == 0 
        fprintf('Progress: %3.0f%%\n', (n / Nt) * 100);
    end
end

%% 5. data extraction & error calculation
if isempty(detections)
    error('No detections made.');
end
res_time_s    = detections(:, 2);        
res_meas_m    = detections(:, 3);        
res_true_m    = detections(:, 4);        
res_error_cm  = (res_meas_m - res_true_m) * 100; 
res_tof_ms    = detections(:, 1) * 1000;

% print stats report
fprintf('\n======================================================\n');
fprintf('              NOISY SIMULATION REPORT                 \n');
fprintf('======================================================\n');
fprintf('Initial Dist.: %.2f m\n', L_start);
fprintf('Wall Velocity: %.2f m/s\n', wall_velocity);
fprintf('PRI:           %.2f ms\n', PRI*1000);
fprintf('Noise Level:   %.3f V\n', noise_level);
fprintf('Threshold:     %.3f V\n', sensor_threshold);
fprintf('-----------------------------------------------------------------------------------\n');
fprintf('Pulse # | Time of Flight | Arrival Time | Measured Dist | True Dist  | Error\n');
fprintf('-----------------------------------------------------------------------------------\n');
for k = 1:size(detections, 1)
    fprintf('   %d    |    %6.2f ms   |   %6.2f ms  |   %.4f m    |  %.4f m  | %.2f cm\n', ...
        k, res_tof_ms(k), res_time_s(k)*1000, res_meas_m(k), res_true_m(k), abs(res_error_cm(k)));
end

figure('Name', 'System Analysis Report');
subplot(3,1,1);
plot(res_time_s, res_meas_m, 'ko-', 'LineWidth', 1.2, 'MarkerSize', 4, 'MarkerFaceColor', 'b'); hold on;
plot(res_time_s, res_true_m, 'g-', 'LineWidth', 2);
grid on; ylabel('Distance [m]'); xlabel('Simulation Time [s]'); 
title('1. Tracking Performance: Measured vs. True Distance');
legend('Sensor Measurement', 'Ground Truth (Actual Wall)', 'Location', 'southwest');

subplot(3,1,2);
stem(res_time_s, res_error_cm, 'filled', 'r', 'LineWidth', 1.5); hold on;
yline(mean(res_error_cm), 'b--', 'Mean Bias');
grid on; ylabel('Error [cm]'); xlabel('Simulation Time [s]');
title(sprintf('2. Measurement Error (Mean Bias: %.2f cm)', mean(res_error_cm)));
legend('Instant Error', 'Systematic Bias'); 

subplot(3,1,3);
plot(res_time_s, res_tof_ms, 'b-d', 'LineWidth', 1.5, 'MarkerFaceColor', 'c');
grid on; xlabel('Simulation Time [s]'); ylabel('ToF [ms]');
title('3. Time of Flight (Raw Sensor Data)');

%% 6. doppler velocity analysis (fft)
fprintf('\n------------------------------------------------------\n');
fprintf('           DOPPLER VELOCITY ANALYSIS                  \n');
fprintf('------------------------------------------------------\n');
Fs = 1/dt; nfft = 8192; % padding for high resolution         

% transmitter profile fft reference
Y_tx = fft(single_pulse, nfft);
P1_tx = abs(Y_tx(1:nfft/2+1) / length(single_pulse)); P1_tx(2:end-1) = 2*P1_tx(2:end-1);

% extract echo signal for frequency analysis
arr_idx = round(detections(1, 1) / dt);
win_size = round((cycles / f0) / dt); 
echo_signal = sensor_readings(max(1, arr_idx-win_size) : min(Nt, arr_idx+win_size));

Y_rx = fft(echo_signal, nfft); 
P1_rx = abs(Y_rx(1:nfft/2+1) / length(echo_signal)); P1_rx(2:end-1) = 2*P1_rx(2:end-1);
f_vec = Fs * (0:(nfft/2)) / nfft; 

% pull frequency peak shift
[~, max_idx] = max(P1_rx); peak_freq = f_vec(max_idx);
measured_shift = peak_freq - f0;

% calculate speed using doppler shift formula
calc_v_mag = (abs(measured_shift) * c) / (peak_freq + f0);
if measured_shift > 0; calculated_velocity = -calc_v_mag; else; calculated_velocity = calc_v_mag; end

fprintf('Base Frequency (f0):     %.0f Hz\n', f0);
fprintf('Measured Peak (f_new):   %.2f Hz\n', peak_freq);
fprintf('Doppler Shift:           %.2f Hz\n', measured_shift);
fprintf('------------------------------------------------------\n');
fprintf('CALCULATED VELOCITY:     %.2f m/s\n', calculated_velocity);
fprintf('ACTUAL WALL VELOCITY:    %.2f m/s\n', wall_velocity);
fprintf('ERROR:                   %.2f m/s\n', abs(calculated_velocity - wall_velocity));

figure('Name', 'Doppler Analysis');
plot(f_vec/1000, P1_tx / max(P1_tx), 'k--', 'LineWidth', 1.2); hold on;
plot(f_vec/1000, P1_rx / max(P1_rx), 'r-', 'LineWidth', 2);
grid on; xlim([38 42]); xlabel('Frequency [kHz]'); ylabel('Normalized Magnitude');
title(sprintf('Doppler Shift Visualization (Delta = %.2f Hz)', measured_shift));
xline(40, 'k-', 'Transmitted (f0)'); xline(peak_freq/1000, 'b--', 'Received (Peak)');
legend('Transmitted Pulse (Ref)', 'Received Echo (Shifted)', 'f0', 'f_{new}', 'Location', 'best');
