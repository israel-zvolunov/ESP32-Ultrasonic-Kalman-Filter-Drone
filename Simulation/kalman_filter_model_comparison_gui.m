function kalman_filter_model_comparison_gui()
  
    % create application window and control board panel
    fig = uifigure('Name', 'UAV Altitude Estimation: Tracking & Error Analysis', 'Position', [50 50 1250 830]);
    pnl = uipanel(fig, 'Title', 'Simulation Control Dashboard', 'Position', [40 640 1170 190], 'FontWeight', 'bold');
   
    % sync slider dragging with text displays and trigger math
    function live_update(src, event, label_handle, format_str)
        label_handle.Text = num2str(event.Value, format_str);
        src.Value = event.Value;
        update_sim();
    end

    % =========================================================
    % Sliders, Labels, Checkboxes
    % =========================================================
    
    % --- Column 1: Flight Dynamics (Initial states and maneuver planning) ---
    uilabel(pnl, 'Text', '1. Flight Dynamics', 'Position', [10 140 150 20], 'FontWeight', 'bold', 'FontSize', 13);
    
    uilabel(pnl, 'Text', 'Initial Altitude [m]', 'Position', [10 100 120 20]);
    sldH = uislider(pnl, 'Limits', [1 8], 'Value', 4, 'Position', [130 110 100 3]);
    valH = uilabel(pnl, 'Text', num2str(sldH.Value, '%.1f'), 'Position', [240 100 40 20], 'FontWeight', 'bold');
    
    uilabel(pnl, 'Text', 'Descent Velocity [m/s]', 'Position', [10 60 130 20]);
    sldV = uislider(pnl, 'Limits', [0.1 5], 'Value', 1.2, 'Position', [130 70 100 3]);
    valV = uilabel(pnl, 'Text', num2str(sldV.Value, '%.2f'), 'Position', [240 60 40 20], 'FontWeight', 'bold');

    uilabel(pnl, 'Text', 'Flare Height [m]', 'Position', [10 20 130 20]);
    sldFlare = uislider(pnl, 'Limits', [0.1 8.0], 'Value', 1.5, 'Position', [130 30 100 3]);
    valFlare = uilabel(pnl, 'Text', num2str(sldFlare.Value, '%.1f'), 'Position', [240 20 40 20], 'FontWeight', 'bold');

    % --- Column 2: Real World Physics (Hidden from the Kalman Filter) ---
    uilabel(pnl, 'Text', '2. Environment (Ground Truth)', 'Position', [300 140 230 20], 'FontWeight', 'bold', 'FontSize', 13, 'FontColor', [0.7 0.1 0.1]);
    
    uilabel(pnl, 'Text', 'Env. Disturbances(\sigma_w)[m/s^2]', 'Position', [300 100 170 20], 'Interpreter', 'tex');
    sldTrueW = uislider(pnl, 'Limits', [0 3.0], 'Value', 0.5, 'Position', [470 110 110 3]);
    valTrueW = uilabel(pnl, 'Text', num2str(sldTrueW.Value, '%.2f'), 'Position', [590 100 40 20], 'FontWeight', 'bold');

    uilabel(pnl, 'Text', 'Sensor Noise (R)[m^2]', 'Position', [300 60 130 20], 'Interpreter', 'tex');
    sldTrueR = uislider(pnl, 'Limits', [0.001 0.5], 'Value', 0.01, 'Position', [470 70 110 3]);
    valTrueR = uilabel(pnl, 'Text', num2str(sldTrueR.Value, '%.3f'), 'Position', [590 60 40 20], 'FontWeight', 'bold');
    
    % --- Column 3: Kalman Filter Tuning (The Algorithm's Assumptions) ---
    uilabel(pnl, 'Text', '3. Kalman Filter (Algorithm)', 'Position', [660 140 200 20], 'FontWeight', 'bold', 'FontSize', 13, 'FontColor', [0 0.3 0.7]);
    
    uilabel(pnl, 'Text', 'Process Noise (\sigma_a)[m/s^2]', 'Position', [660 100 150 20], 'Interpreter', 'tex');
    sldFilterQ = uislider(pnl, 'Limits', [0.01 5.0], 'Value', 1.0, 'Position', [830 110 100 3]);
    valFilterQ = uilabel(pnl, 'Text', num2str(sldFilterQ.Value, '%.2f'), 'Position', [940 100 40 20], 'FontWeight', 'bold');

    uilabel(pnl, 'Text', 'Meas. Covariance (R)[m^2]', 'Position', [660 60 160 25], 'Interpreter', 'tex');
    sldFilterR = uislider(pnl, 'Limits', [0.001 0.5], 'Value', 0.05, 'Position', [830 70 100 3]);
    valFilterR = uilabel(pnl, 'Text', num2str(sldFilterR.Value, '%.3f'), 'Position', [940 60 40 20], 'FontWeight', 'bold');

    % --- Column 4: Checkboxes & Reset Controls ---
    % 1. Master Control
    uibutton(pnl, 'Text', 'Reset Simulation', 'Position', [990 140 150 25], 'ButtonPushedFcn', @(btn,event) reset_sim(), 'BackgroundColor', [0.9 0.9 0.9]);
    
    % 2. Physical & Sensor Scenarios (Disturbances)
    cbFreeFall = uicheckbox(pnl, 'Text', 'Free Fall Drop (-9.81 m/s²)', 'Value', 0, 'Position', [990 115 180 20], 'FontWeight', 'bold', 'ValueChangedFcn', @(s,e) update_sim());
    cbOutage = uicheckbox(pnl, 'Text', 'Sensor Outage (1 sec)', 'Value', 0, 'Position', [990 95 160 20], 'FontWeight', 'bold', 'FontColor', [0.4 0 0.6], 'ValueChangedFcn', @(s,e) update_sim());
    cbSpikes = uicheckbox(pnl, 'Text', 'Inject Spikes', 'Value', 1, 'Position', [990 75 150 20], 'FontWeight', 'bold', 'FontColor', 'r', 'ValueChangedFcn', @(s,e) update_sim());
    
    % 3. Display Controls (Visualization)
    cbMeas = uicheckbox(pnl, 'Text', 'Show Measurements', 'Value', 1, 'Position', [990 50 150 20], 'FontColor', [0.8 0.4 0], 'ValueChangedFcn', @(s,e) update_sim());
    cbCV   = uicheckbox(pnl, 'Text', 'Show CV Model', 'Value', 1, 'Position', [990 30 120 20], 'FontColor', 'b', 'ValueChangedFcn', @(s,e) update_sim());
    cbCA   = uicheckbox(pnl, 'Text', 'Show CA Model', 'Value', 1, 'Position', [990 10 120 20], 'FontColor', 'm', 'ValueChangedFcn', @(s,e) update_sim());
   
    % Assign callbacks for continuous dragging execution
    sldH.ValueChangingFcn = @(src, event) live_update(src, event, valH, '%.1f');
    sldV.ValueChangingFcn = @(src, event) live_update(src, event, valV, '%.2f');
    sldFlare.ValueChangingFcn = @(src, event) live_update(src, event, valFlare, '%.1f');
    sldTrueW.ValueChangingFcn = @(src, event) live_update(src, event, valTrueW, '%.2f');
    sldTrueR.ValueChangingFcn = @(src, event) live_update(src, event, valTrueR, '%.3f');
    sldFilterQ.ValueChangingFcn = @(src, event) live_update(src, event, valFilterQ, '%.2f');
    sldFilterR.ValueChangingFcn = @(src, event) live_update(src, event, valFilterR, '%.3f');

    % restore original baseline values
    function reset_sim()
        sldH.Value = 4.0; valH.Text = '4.0'; sldV.Value = 1.2; valV.Text = '1.20';
        sldFlare.Value = 1.5; valFlare.Text = '1.5'; sldTrueW.Value = 0.5; valTrueW.Text = '0.50';
        sldTrueR.Value = 0.01; valTrueR.Text = '0.010'; 
        sldFilterQ.Value = 1.0; valFilterQ.Text = '1.00'; sldFilterR.Value = 0.05; valFilterR.Text = '0.050';
        cbMeas.Value = 1; cbCV.Value = 1; cbCA.Value = 1; cbSpikes.Value = 1;
        cbFreeFall.Value = 0; cbOutage.Value = 0;
        update_sim();
    end

    % create four empty graph windows with permanent layout positions
    ax_alt = uiaxes(fig, 'Position', [40 330 550 290]);      
    ax_vel = uiaxes(fig, 'Position', [40 20 550 290]);       
    ax_err_alt = uiaxes(fig, 'Position', [650 330 550 290]); 
    ax_err_vel = uiaxes(fig, 'Position', [650 20 550 290]);  
    
    % --- Setup Altitude Axes ---
    hold(ax_alt, 'on'); grid(ax_alt, 'on');
    h_fill_alt = fill(ax_alt, NaN, NaN, [0.9 0.9 0.8], 'EdgeColor', 'none', 'HandleVisibility', 'off');
    yline(ax_alt, 0, 'Color', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    h_true_alt = plot(ax_alt, NaN, NaN, 'g', 'LineWidth', 2.5, 'DisplayName', 'True Altitude');
    h_meas_alt = scatter(ax_alt, NaN, NaN, 10, 'r', 'filled', 'MarkerFaceAlpha', 0.4, 'DisplayName', 'Sensor Output');
    h_cv_alt = plot(ax_alt, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'CV Estimate');
    h_ca_alt = plot(ax_alt, NaN, NaN, 'm', 'LineWidth', 1.5, 'DisplayName', 'CA Estimate');
    ylabel(ax_alt, 'Altitude [m]'); title(ax_alt, 'Altitude Tracking'); legend(ax_alt, 'Location', 'northeast');

    % --- Setup Velocity Axes ---
    hold(ax_vel, 'on'); grid(ax_vel, 'on');
    yline(ax_vel, 0, 'Color', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    h_true_vel = plot(ax_vel, NaN, NaN, 'g', 'LineWidth', 2.5, 'DisplayName', 'True Velocity');
    h_cv_vel = plot(ax_vel, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'CV Vel Est');
    h_ca_vel = plot(ax_vel, NaN, NaN, 'm', 'LineWidth', 1.5, 'DisplayName', 'CA Vel Est');
    ylabel(ax_vel, 'Velocity [m/s]'); xlabel(ax_vel, 'Time [s]'); title(ax_vel, 'Velocity Estimation'); legend(ax_vel, 'Location', 'southeast');

    % --- Setup Altitude Error Axes ---
    hold(ax_err_alt, 'on'); grid(ax_err_alt, 'on');
    yline(ax_err_alt, 0, 'Color', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
    h_err_meas = scatter(ax_err_alt, NaN, NaN, 10, 'r', 'filled', 'MarkerFaceAlpha', 0.4, 'DisplayName', 'Sensor Error');
    h_err_cv_alt = plot(ax_err_alt, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'CV Alt Error');
    h_err_ca_alt = plot(ax_err_alt, NaN, NaN, 'm', 'LineWidth', 1.5, 'DisplayName', 'CA Alt Error');
    ylabel(ax_err_alt, 'Error [m]'); title(ax_err_alt, 'Altitude Error vs. Time (e = Est - True)'); legend(ax_err_alt, 'Location', 'northeast');

    % --- Setup Velocity Error Axes ---
    hold(ax_err_vel, 'on'); grid(ax_err_vel, 'on');
    yline(ax_err_vel, 0, 'Color', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
    h_err_cv_vel = plot(ax_err_vel, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'CV Vel Error');
    h_err_ca_vel = plot(ax_err_vel, NaN, NaN, 'm', 'LineWidth', 1.5, 'DisplayName', 'CA Vel Error');
    ylabel(ax_err_vel, 'Error [m/s]'); xlabel(ax_err_vel, 'Time [s]'); title(ax_err_vel, 'Velocity Error vs. Time'); legend(ax_err_vel, 'Location', 'southeast');

    update_sim();

    % =========================================================
    % Math, Physics, and Estimation
    % =========================================================
    function update_sim()
        rng(42); % force same noise values during real-time tracking
        
        h0 = sldH.Value; 
        v0 = sldV.Value; 
        flare_height = sldFlare.Value;
        
        % make sure flare doesn't exceed beginning height
        if flare_height > h0 - 0.2
            flare_height = max(0.1, h0 - 0.2); 
        end
        
        true_w = sldTrueW.Value; 
        true_R = sldTrueR.Value;   
        Q_factor = sldFilterQ.Value; 
        R_filter = sldFilterR.Value; 
        
        dt = 1/30; % 33.3ms tracking sample rate
        t_land = (h0 / v0) * 2.0; 
        t = 0:dt:(t_land + 1.5); 
        N = length(t);
        
        z_true = zeros(1, N); 
        v_true = zeros(1, N); 
        a_true = zeros(1, N);
        z_true(1) = h0; 
        v_true(1) = -v0; 
        
        % ----------------------------------------------------
        % 1. PHYSICAL MODEL: Generate Ground Truth Trajectory
        % ----------------------------------------------------
        for k = 2:N
            if z_true(k-1) <= 0
                % lock everything to zero once landed
                z_true(k) = 0; v_true(k) = 0; a_true(k) = 0;
            else
                if cbFreeFall.Value == 1
                    if z_true(k-1) > flare_height
                        a_det = -9.81; % free fall descent acceleration
                    else
                        % hard braking mechanics to stop drop safely at zero
                        a_det = (v_true(k-1)^2) / (2 * max(0.05, z_true(k-1)));
                    end
                else
                    % normal landing profile trajectory
                    a_det = (z_true(k-1) <= flare_height) * (v0^2 / (2 * flare_height));
                end
                
                % apply process disturbance noise
                a_true(k) = a_det + true_w * randn(); 
                v_true(k) = v_true(k-1) + a_true(k) * dt;   
                
                if cbFreeFall.Value == 0 && v_true(k) > -0.05 && z_true(k-1) > 0
                    v_true(k) = -0.05; 
                end
                
                z_true(k) = z_true(k-1) + v_true(k-1) * dt + 0.5 * a_true(k) * dt^2;
                if z_true(k) < 0, z_true(k) = 0; v_true(k) = 0; end
            end
        end
        
        % ----------------------------------------------------
        % 2. SENSOR MODEL: Generate Noisy Measurements
        % ----------------------------------------------------
        z_meas = z_true + sqrt(true_R) * randn(1, N);
        
        % simulate ultrasonic multipath reflections (outlier spikes)
        if cbSpikes.Value
            valid_range = max(1, N-15); 
            spike_idx = randperm(valid_range, min(6, valid_range)) + 10;
            z_meas(spike_idx) = z_meas(spike_idx) + (rand(1,length(spike_idx))-0.5)*4.0; 
        end

        % mimic sonar hardware blackout dropouts
        if cbOutage.Value
            outage_idx = t >= 1.0 & t <= 2.0; 
            z_meas(outage_idx) = NaN;
        end
        
        % ----------------------------------------------------
        % 3. KALMAN FILTER SETUP (Initialization)
        % ----------------------------------------------------
        % constant velocity initialization matrices
        x_cv = [z_meas(1); 0]; 
        P_cv = eye(2); 
        A_cv = [1 dt; 0 1]; 
        H_cv = [1 0];       
        Q_cv = Q_factor^2 * [(dt^4)/4 (dt^3)/2; (dt^3)/2 dt^2]; 
        
        % constant acceleration initialization matrices
        x_ca = [z_meas(1); 0; 0]; 
        P_ca = eye(3); 
        A_ca = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1]; 
        H_ca = [1 0 0];
        G_ca = [dt^2/2; dt; 1]; 
        Q_ca = (Q_factor*0.5)^2 * (G_ca * G_ca'); 
        
        z_est_cv = zeros(1, N); v_est_cv = zeros(1, N);
        z_est_ca = zeros(1, N); v_est_ca = zeros(1, N);

        % ----------------------------------------------------
        % 4. KALMAN FILTER LOOP (Predict & Update)
        % ----------------------------------------------------
        for k = 1:N
            z_est_cv(k) = x_cv(1); v_est_cv(k) = x_cv(2);
            z_est_ca(k) = x_ca(1); v_est_ca(k) = x_ca(2);
            z = z_meas(k); 
            
            % cv model state recursion steps
            if ~isnan(z)
                y_cv = z - H_cv * x_cv;                  
                S_cv = H_cv * P_cv * H_cv' + R_filter;   
                K_cv = (P_cv * H_cv') / S_cv;            
                x_cv = x_cv + K_cv * y_cv;               
                P_cv = (eye(2) - K_cv * H_cv) * P_cv;    
            end
            x_cv = A_cv * x_cv;                          
            P_cv = A_cv * P_cv * A_cv' + Q_cv;           
            
            % ca model state recursion steps
            if ~isnan(z)
                y_ca = z - H_ca * x_ca; 
                S_ca = H_ca * P_ca * H_ca' + R_filter; 
                K_ca = (P_ca * H_ca') / S_ca; 
                x_ca = x_ca + K_ca * y_ca; 
                P_ca = (eye(3) - K_ca * H_ca) * P_ca;
            end
            x_ca = A_ca * x_ca; 
            P_ca = A_ca * P_ca * A_ca' + Q_ca; 
        end
        
        % ----------------------------------------------------
        % 5. PERFORMANCE METRICS (Error Calculation)
        % ----------------------------------------------------
        err_meas_t   = z_meas - z_true;
        err_cv_alt_t = z_est_cv - z_true;
        err_ca_alt_t = z_est_ca - z_true;
        err_cv_vel_t = v_est_cv - v_true;
        err_ca_vel_t = v_est_ca - v_true;

        % evaluate mean absolute error metrics only during airborne phase
        flight_idx = find(z_true > 0.1); 
        if isempty(flight_idx), flight_idx = 1:N; end
        err_meas_mae   = mean(abs(err_meas_t(flight_idx)));
        err_cv_alt_mae = mean(abs(err_cv_alt_t(flight_idx)));
        err_ca_alt_mae = mean(abs(err_ca_alt_t(flight_idx)));
        err_cv_vel_mae = mean(abs(err_cv_vel_t(flight_idx)));
        err_ca_vel_mae = mean(abs(err_ca_vel_t(flight_idx)));

        % ----------------------------------------------------
        % 6. GRAPHICS RENDERING
        % ----------------------------------------------------
        vis_state = {'off', 'on'}; 
        
        h_fill_alt.XData = [0 t(end) t(end) 0]; h_fill_alt.YData = [-2 -2 0 0];
        h_true_alt.XData = t; h_true_alt.YData = z_true;
        h_meas_alt.XData = t; h_meas_alt.YData = z_meas;
        h_cv_alt.XData = t;   h_cv_alt.YData = z_est_cv;
        h_ca_alt.XData = t;   h_ca_alt.YData = z_est_ca;
        
        h_meas_alt.Visible = vis_state{cbMeas.Value + 1};
        h_cv_alt.Visible   = vis_state{cbCV.Value + 1};
        h_ca_alt.Visible   = vis_state{cbCA.Value + 1};
        
        title(ax_alt, sprintf('Altitude Tracking [MAE - Meas: %.2f m | CV: %.2f m | CA: %.2f m]', err_meas_mae, err_cv_alt_mae, err_ca_alt_mae));
        ylim(ax_alt, [-1 h0+2]); xlim(ax_alt, [0 t(end)]); 

        h_true_vel.XData = t; h_true_vel.YData = v_true;
        h_cv_vel.XData = t;   h_cv_vel.YData = v_est_cv;
        h_ca_vel.XData = t;   h_ca_vel.YData = v_est_ca;
        
        h_cv_vel.Visible = vis_state{cbCV.Value + 1};
        h_ca_vel.Visible = vis_state{cbCA.Value + 1};
        
        max_brake_g = max(a_true) / 9.81;
        title(ax_vel, sprintf('Velocity [MAE - CV: %.2f m/s | CA: %.2f m/s] | Max Brake: %.1f G', err_cv_vel_mae, err_ca_vel_mae, max_brake_g));
        
        min_v = min(-v0-1.5, min(v_true) - 2);
        ylim(ax_vel, [min_v 2]); xlim(ax_vel, [0 t(end)]);

        h_err_meas.XData = t  ; h_err_meas.YData = err_meas_t;
        h_err_cv_alt.XData = t; h_err_cv_alt.YData = err_cv_alt_t;
        h_err_ca_alt.XData = t; h_err_ca_alt.YData = err_ca_alt_t;
        
        h_err_meas.Visible   = vis_state{cbMeas.Value + 1};
        h_err_cv_alt.Visible = vis_state{cbCV.Value + 1};
        h_err_ca_alt.Visible = vis_state{cbCA.Value + 1};
        ylim(ax_err_alt, [-1.5 1.5]); xlim(ax_err_alt, [0 t(end)]);

        h_err_cv_vel.XData = t; h_err_cv_vel.YData = err_cv_vel_t;
        h_err_ca_vel.XData = t; h_err_ca_vel.YData = err_ca_vel_t;
        
        h_err_cv_vel.Visible = vis_state{cbCV.Value + 1};
        h_err_ca_vel.Visible = vis_state{cbCA.Value + 1};
        ylim(ax_err_vel, [-3 3]); xlim(ax_err_vel, [0 t(end)]);
        
        drawnow limitrate;
    end
end