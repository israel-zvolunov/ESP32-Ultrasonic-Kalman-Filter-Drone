% ========================================================
% Ground Station Dashboard
% ========================================================
clear; close all; clc;
udpPort = 4210; 

try
    % open socket for incoming esp32 data
    u = udpport("datagram", "LocalPort", udpPort);
    flush(u); % wipe buffer junk
    disp('Listening for telemetry on UDP port 4210...');
catch
    error('Could not open UDP port. Is it already open in another script?');
end

% --- Graph Setup ---
% window size buffer for plotting
windowSize = 150;
x_axis = 1:windowSize;

y_raw = nan(1, windowSize);
y_kalman = nan(1, windowSize);
y_lidar = nan(1, windowSize); 
err_raw_arr = nan(1, windowSize);
err_kalman_arr = nan(1, windowSize);

fig = figure('Name', 'Flight Dashboard & Error Analysis', 'NumberTitle', 'off', 'Color', 'w', 'Position', [100, 50, 950, 750]);

% Top Graph: Altitude tracking
ax1 = subplot(2, 1, 1);
hold(ax1, 'on');
h_raw = plot(ax1, x_axis, y_raw, 'r-', 'MarkerSize', 8, 'DisplayName', 'Raw');
h_kalman = plot(ax1, x_axis, y_kalman, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Kalman');
h_lidar = plot(ax1, x_axis, y_lidar, 'k-', 'LineWidth', 1.5, 'DisplayName', 'LiDAR (True)'); 
legend(ax1, 'Location', 'northeast'); grid(ax1, 'on');
ylabel(ax1, 'Distance (cm)'); title(ax1, 'Altitude vs. Time');
xlim(ax1, [1, windowSize]); ylim(ax1, [0, 840]); 

% Bottom Graph: Error over time
ax2 = subplot(2, 1, 2);
hold(ax2, 'on');
h_err_raw = plot(ax2, x_axis, err_raw_arr, 'r-', 'LineWidth', 1, 'DisplayName', 'Raw Error');
h_err_kalman = plot(ax2, x_axis, err_kalman_arr, 'b-', 'LineWidth', 2, 'DisplayName', 'Kalman Error');
legend(ax2, 'Location', 'northeast'); grid(ax2, 'on');
xlabel(ax2, 'Samples'); ylabel(ax2, 'Error (cm)'); title(ax2, 'Absolute Error');
xlim(ax2, [1, windowSize]); ylim(ax2, [0, 400]);

% ui elements for text display
txtRmse = uicontrol('Style', 'text', 'String', 'Calculating RMSE...', ...
    'Units', 'normalized', 'Position', [0.1, 0.02, 0.5, 0.05], ...
    'FontSize', 14, 'FontWeight', 'bold', 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
    
txtStatus = uicontrol('Style', 'text', 'String', 'STATUS: WAITING', ...
    'Units', 'normalized', 'Position', [0.65, 0.02, 0.25, 0.06], ...
    'FontSize', 16, 'FontWeight', 'bold', 'ForegroundColor', 'w', 'BackgroundColor', 'k');

plotCounter = 0;
fpsTimer = tic;
frameCount = 0;

txtFPS = uicontrol('Style', 'text', 'String', 'Rate: -- Hz', ...
    'Units', 'normalized', 'Position', [0.4, 0.02, 0.2, 0.05], ...
    'FontSize', 14, 'FontWeight', 'bold', 'BackgroundColor', 'w');

% --- Main Loop ---
while ishandle(fig)
    numPkts = u.NumDatagramsAvailable;
    if numPkts > 0
        frameCount = frameCount + numPkts;

        % dump backlog if network lags
        if numPkts > 10
            flush(u);
        else
            % parse data packets
            datagrams = read(u, numPkts);
            latestData = datagrams(end).Data;
            
            % break payload string by tabs
            vals = str2double(split(string(char(latestData)), char(9))); 
            
            if numel(vals) == 3 && ~any(isnan(vals))
                raw_val = vals(1); kalman_val = vals(2); lidar_val = vals(3);
                
                % get absolute errors
                curr_err_raw = abs(raw_val - lidar_val);
                curr_err_kalman = abs(kalman_val - lidar_val);
                
                % slide display window
                y_raw = [y_raw(2:end), raw_val];
                y_kalman = [y_kalman(2:end), kalman_val];
                y_lidar = [y_lidar(2:end), lidar_val];
                err_raw_arr = [err_raw_arr(2:end), curr_err_raw];
                err_kalman_arr = [err_kalman_arr(2:end), curr_err_kalman];
                
                % update dynamic plot paths
                h_raw.YData = y_raw; h_kalman.YData = y_kalman; h_lidar.YData = y_lidar;
                h_err_raw.YData = err_raw_arr; h_err_kalman.YData = err_kalman_arr;
                
                % evaluate statistics
                rmse_raw = sqrt(mean((y_raw - y_lidar).^2, 'omitnan'));
                rmse_kalman = sqrt(mean((y_kalman - y_lidar).^2, 'omitnan'));
                txtRmse.String = sprintf('RMSE -> Raw: %.1f cm | Kalman: %.1f cm', rmse_raw, rmse_kalman);
                
                % distance threshold evaluation
                if kalman_val > 100
                    txtStatus.String = 'SAFE (> 1m)'; txtStatus.BackgroundColor = [0.2 0.8 0.2];
                else
                    txtStatus.String = 'WARNING: LOW!'; txtStatus.BackgroundColor = [0.9 0.1 0.1];
                end
                
                % throttle render calls to save cpu
                plotCounter = plotCounter + 1;
                if mod(plotCounter, 2) == 0 
                    drawnow limitrate;
                end
            end
        end
        
        % compute packet arrival frequency once per second
        elapsedTime = toc(fpsTimer);
        if elapsedTime >= 1.0
            txtFPS.String = sprintf('Rate: %d Hz', frameCount);
            frameCount = 0;
            fpsTimer = tic;
        end
    end
end
clear u;
