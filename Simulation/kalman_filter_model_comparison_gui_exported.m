classdef kalman_filter_model_comparison_gui_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        SimulationControlDashboardPanel  matlab.ui.container.Panel
        cbCA                            matlab.ui.control.CheckBox
        cbCV                            matlab.ui.control.CheckBox
        cbMeas                          matlab.ui.control.CheckBox
        cbSpikes                        matlab.ui.control.CheckBox
        cbOutage                        matlab.ui.control.CheckBox
        cbFreeFall                      matlab.ui.control.CheckBox
        ResetButton                     matlab.ui.control.Button
        valFilterR                      matlab.ui.control.Label
        FilterRSlider                   matlab.ui.control.Slider
        MeasCovarianceRm2Label          matlab.ui.control.Label
        valFilterQ                      matlab.ui.control.Label
        FilterQSlider                   matlab.ui.control.Slider
        ProcessNoisesigma_ams2Label     matlab.ui.control.Label
        KalmanFilterAlgorithmLabel      matlab.ui.control.Label
        TrueRSlider                     matlab.ui.control.Slider
        valTrueR                        matlab.ui.control.Label
        SensorNoiseRm2Label             matlab.ui.control.Label
        TrueWSlider                     matlab.ui.control.Slider
        valTrueW                        matlab.ui.control.Label
        EnvDisturbancessigma_wms2Label  matlab.ui.control.Label
        EnvironmentGroundTruthLabel     matlab.ui.control.Label
        valFlare                        matlab.ui.control.Label
        FlareHeightmLabel               matlab.ui.control.Label
        FlareHeightSlider               matlab.ui.control.Slider
        DescentVelocitySlider           matlab.ui.control.Slider
        valV                            matlab.ui.control.Label
        DescentVelocitymsLabel          matlab.ui.control.Label
        valH                            matlab.ui.control.Label
        InitialAltitudeSlider           matlab.ui.control.Slider
        InitialAltitudemLabel           matlab.ui.control.Label
        FlightDynamicsLabel             matlab.ui.control.Label
        VelErrorAxes                    matlab.ui.control.UIAxes
        AltErrorAxes                    matlab.ui.control.UIAxes
        VelocityAxes                    matlab.ui.control.UIAxes
        AltitudeAxes                    matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        h_fill_alt
        h_true_alt
        h_meas_alt
        h_cv_alt
        h_ca_alt
        h_true_vel
        h_cv_vel
        h_ca_vel
        h_err_meas
        h_err_cv_alt
        h_err_ca_alt
        h_err_cv_vel
        h_err_ca_vel
    end
    
        methods (Access = private)
        
        function update_sim(app)
            rng(42); % fix seed for repeatable noise
            
            % get values from gui sliders
            h0 = app.InitialAltitudeSlider.Value; 
            v0 = app.DescentVelocitySlider.Value; 
            flare_height = app.FlareHeightSlider.Value;
            
            % flare check
            if flare_height > h0 - 0.2
                flare_height = max(0.1, h0 - 0.2); 
            end
            
            true_w   = app.TrueWSlider.Value; 
            true_R   = app.TrueRSlider.Value;   
            Q_factor = app.FilterQSlider.Value; 
            R_filter = app.FilterRSlider.Value; 
            
            dt = 1/30; % 33.3ms sampling time
            t_land = (h0 / v0) * 2.0; 
            t = 0:dt:(t_land + 1.5); 
            N = length(t);
            
            z_true = zeros(1, N); 
            v_true = zeros(1, N); 
            a_true = zeros(1, N);
            z_true(1) = h0; 
            v_true(1) = -v0; 
            
            % 1. true uav trajectory generation
            for k = 2:N
                if z_true(k-1) <= 0
                    % uav landed on the ground
                    z_true(k) = 0; v_true(k) = 0; a_true(k) = 0;
                else
                    if app.cbFreeFall.Value == 1
                        if z_true(k-1) > flare_height
                            a_det = -9.81; 
                        else
                            % hard brake before impact
                            a_det = (v_true(k-1)^2) / (2 * max(0.05, z_true(k-1)));
                        end
                    else
                        % normal landing profile
                        a_det = (z_true(k-1) <= flare_height) * (v0^2 / (2 * flare_height));
                    end
                    
                    % add process noise to acceleration
                    a_true(k) = a_det + true_w * randn(); 
                    v_true(k) = v_true(k-1) + a_true(k) * dt;   
                    
                    if app.cbFreeFall.Value == 0 && v_true(k) > -0.05 && z_true(k-1) > 0
                        v_true(k) = -0.05; 
                    end
                    
                    z_true(k) = z_true(k-1) + v_true(k-1) * dt + 0.5 * a_true(k) * dt^2;
                    if z_true(k) < 0, z_true(k) = 0; v_true(k) = 0; end
                end
            end
            
            % 2. generate noisy measurements
            z_meas = z_true + sqrt(true_R) * randn(1, N);
            
            % add multipath reflection spikes
            if app.cbSpikes.Value
                valid_range = max(1, N-15); 
                spike_idx = randperm(valid_range, min(6, valid_range)) + 10;
                z_meas(spike_idx) = z_meas(spike_idx) + (rand(1,length(spike_idx))-0.5)*4.0; 
            end

            % simulate sensor outage
            if app.cbOutage.Value
                outage_idx = t >= 1.0 & t <= 2.0; 
                z_meas(outage_idx) = NaN;
            end
            
            % 3. kalman filter matrices setup
            % constant velocity (cv) model matrices
            x_cv = [z_meas(1); 0]; 
            P_cv = eye(2); 
            A_cv = [1 dt; 0 1]; 
            H_cv = [1 0];       
            Q_cv = Q_factor^2 * [(dt^4)/4 (dt^3)/2; (dt^3)/2 dt^2]; 
            
            % constant acceleration (ca) model matrices
            x_ca = [z_meas(1); 0; 0]; 
            P_ca = eye(3); 
            A_ca = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1]; 
            H_ca = [1 0 0];
            G_ca = [dt^2/2; dt; 1]; 
            Q_ca = (Q_factor*0.5)^2 * (G_ca * G_ca'); 
            
            z_est_cv = zeros(1, N); v_est_cv = zeros(1, N);
            z_est_ca = zeros(1, N); v_est_ca = zeros(1, N);

            % 4. main kalman loop
            for k = 1:N
                z_est_cv(k) = x_cv(1); v_est_cv(k) = x_cv(2);
                z_est_ca(k) = x_ca(1); v_est_ca(k) = x_ca(2);
                z = z_meas(k); 
                
                % cv filter step
                if ~isnan(z)
                    y_cv = z - H_cv * x_cv;                  
                    S_cv = H_cv * P_cv * H_cv' + R_filter;   
                    K_cv = (P_cv * H_cv') / S_cv;            
                    x_cv = x_cv + K_cv * y_cv;               
                    P_cv = (eye(2) - K_cv * H_cv) * P_cv;    
                end
                x_cv = A_cv * x_cv;                          
                P_cv = A_cv * P_cv * A_cv' + Q_cv;           
                
                % ca filter step
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
            
            % 5. error calculations
            err_meas_t   = z_meas - z_true;
            err_cv_alt_t = z_est_cv - z_true;
            err_ca_alt_t = z_est_ca - z_true;
            err_cv_vel_t = v_est_cv - v_true;
            err_ca_vel_t = v_est_ca - v_true;

            % compute mae only during flight phase
            flight_idx = find(z_true > 0.1); 
            if isempty(flight_idx), flight_idx = 1:N; end
            err_meas_mae   = mean(abs(err_meas_t(flight_idx)), 'omitnan');
            err_cv_alt_mae = mean(abs(err_cv_alt_t(flight_idx)));
            err_ca_alt_mae = mean(abs(err_ca_alt_t(flight_idx)));
            err_cv_vel_mae = mean(abs(err_cv_vel_t(flight_idx)));
            err_ca_vel_mae = mean(abs(err_ca_vel_t(flight_idx)));

            % 6. plot results
            vis_state = {'off', 'on'}; 
            
            % update altitude plots
            app.h_fill_alt.XData = [0 t(end) t(end) 0]; app.h_fill_alt.YData = [-2 -2 0 0];
            app.h_true_alt.XData = t; app.h_true_alt.YData = z_true;
            app.h_meas_alt.XData = t; app.h_meas_alt.YData = z_meas;
            app.h_cv_alt.XData = t;   app.h_cv_alt.YData = z_est_cv;
            app.h_ca_alt.XData = t;   app.h_ca_alt.YData = z_est_ca;
            
            app.h_meas_alt.Visible = vis_state{app.cbMeas.Value + 1};
            app.h_cv_alt.Visible   = vis_state{app.cbCV.Value + 1};
            app.h_ca_alt.Visible   = vis_state{app.cbCA.Value + 1};
            
            title(app.AltitudeAxes, sprintf('Altitude Tracking [MAE - Meas: %.2f m | CV: %.2f m | CA: %.2f m]', err_meas_mae, err_cv_alt_mae, err_ca_alt_mae));
            ylim(app.AltitudeAxes, [-1 h0+2]); xlim(app.AltitudeAxes, [0 t(end)]); 

            % update velocity plots
            app.h_true_vel.XData = t; app.h_true_vel.YData = v_true;
            app.h_cv_vel.XData = t;   app.h_cv_vel.YData = v_est_cv;
            app.h_ca_vel.XData = t;   app.h_ca_vel.YData = v_est_ca;
            
            app.h_cv_vel.Visible = vis_state{app.cbCV.Value + 1};
            app.h_ca_vel.Visible = vis_state{app.cbCA.Value + 1};
            
            max_brake_g = max(a_true) / 9.81;
            title(app.VelocityAxes, sprintf('Velocity [MAE - CV: %.2f m/s | CA: %.2f m/s] | Max Brake: %.1f G', err_cv_vel_mae, err_ca_vel_mae, max_brake_g));
            
            min_v = min(-v0-1.5, min(v_true) - 2);
            ylim(app.VelocityAxes, [min_v 2]); xlim(app.VelocityAxes, [0 t(end)]);

            % update altitude error plots
            app.h_err_meas.XData = t;   app.h_err_meas.YData = err_meas_t;
            app.h_err_cv_alt.XData = t; app.h_err_cv_alt.YData = err_cv_alt_t;
            app.h_err_ca_alt.XData = t; app.h_err_ca_alt.YData = err_ca_alt_t;
            
            app.h_err_meas.Visible   = vis_state{app.cbMeas.Value + 1};
            app.h_err_cv_alt.Visible = vis_state{app.cbCV.Value + 1};
            app.h_err_ca_alt.Visible = vis_state{app.cbCA.Value + 1};
            ylim(app.AltErrorAxes, [-1.5 1.5]); xlim(app.AltErrorAxes, [0 t(end)]);

            % update velocity error plots
            app.h_err_cv_vel.XData = t; app.h_err_cv_vel.YData = err_cv_vel_t;
            app.h_err_ca_vel.XData = t; app.h_err_ca_vel.YData = err_ca_vel_t;
            
            app.h_err_cv_vel.Visible = vis_state{app.cbCV.Value + 1};
            app.h_err_ca_vel.Visible = vis_state{app.cbCA.Value + 1};
            ylim(app.VelErrorAxes, [-3 3]); xlim(app.VelErrorAxes, [0 t(end)]);
        end
    end

    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % --- Altitude Tracking ---
            hold(app.AltitudeAxes, 'on'); grid(app.AltitudeAxes, 'on');
            app.h_fill_alt = fill(app.AltitudeAxes, NaN, NaN, [0.9 0.9 0.8], 'EdgeColor', 'none', 'HandleVisibility', 'off');
            yline(app.AltitudeAxes, 0, 'Color', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            app.h_true_alt = plot(app.AltitudeAxes, NaN, NaN, 'g', 'LineWidth', 2.5, 'DisplayName', 'True Altitude');
            app.h_meas_alt = scatter(app.AltitudeAxes, NaN, NaN, 10, 'r', 'filled', 'MarkerFaceAlpha', 0.4, 'DisplayName', 'Sensor Output');
            app.h_cv_alt = plot(app.AltitudeAxes, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'CV Estimate');
            app.h_ca_alt = plot(app.AltitudeAxes, NaN, NaN, 'm', 'LineWidth', 1.5, 'DisplayName', 'CA Estimate');
            ylabel(app.AltitudeAxes, 'Altitude [m]'); title(app.AltitudeAxes, 'Altitude Tracking'); legend(app.AltitudeAxes, 'Location', 'northeast');

            % --- Velocity Estimation ---
            hold(app.VelocityAxes, 'on'); grid(app.VelocityAxes, 'on');
            yline(app.VelocityAxes, 0, 'Color', 'k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            app.h_true_vel = plot(app.VelocityAxes, NaN, NaN, 'g', 'LineWidth', 2.5, 'DisplayName', 'True Velocity');
            app.h_cv_vel = plot(app.VelocityAxes, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'CV Vel Est');
            app.h_ca_vel = plot(app.VelocityAxes, NaN, NaN, 'm', 'LineWidth', 1.5, 'DisplayName', 'CA Vel Est');
            ylabel(app.VelocityAxes, 'Velocity [m/s]'); xlabel(app.VelocityAxes, 'Time [s]'); title(app.VelocityAxes, 'Velocity Estimation'); legend(app.VelocityAxes, 'Location', 'southeast');

            % --- Altitude Error vs. Time ---
            hold(app.AltErrorAxes, 'on'); grid(app.AltErrorAxes, 'on');
            yline(app.AltErrorAxes, 0, 'Color', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            app.h_err_meas = scatter(app.AltErrorAxes, NaN, NaN, 10, 'r', 'filled', 'MarkerFaceAlpha', 0.4, 'DisplayName', 'Sensor Error');
            app.h_err_cv_alt = plot(app.AltErrorAxes, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'CV Alt Error');
            app.h_err_ca_alt = plot(app.AltErrorAxes, NaN, NaN, 'm', 'LineWidth', 1.5, 'DisplayName', 'CA Alt Error');
            ylabel(app.AltErrorAxes, 'Error [m]'); title(app.AltErrorAxes, 'Altitude Error vs. Time (e = Est - True)'); legend(app.AltErrorAxes, 'Location', 'northeast');

            % --- Velocity Error vs. Time ---
            hold(app.VelErrorAxes, 'on'); grid(app.VelErrorAxes, 'on');
            yline(app.VelErrorAxes, 0, 'Color', 'k', 'LineWidth', 2, 'HandleVisibility', 'off');
            app.h_err_cv_vel = plot(app.VelErrorAxes, NaN, NaN, 'b', 'LineWidth', 1.5, 'DisplayName', 'CV Vel Error');
            app.h_err_ca_vel = plot(app.VelErrorAxes, NaN, NaN, 'm', 'LineWidth', 1.5, 'DisplayName', 'CA Vel Error');
            ylabel(app.VelErrorAxes, 'Error [m/s]'); xlabel(app.VelErrorAxes, 'Time [s]'); title(app.VelErrorAxes, 'Velocity Error vs. Time'); legend(app.VelErrorAxes, 'Location', 'southeast');

            app.update_sim(); 
        end

        % Value changing function: InitialAltitudeSlider
        function InitialAltitudeSliderValueChanging(app, event)
            changingValue = event.Value;
            app.valH.Text = num2str(changingValue, '%.1f'); % update label text
            app.update_sim(); % re-run tracking simulation
            
        end

        % Value changing function: DescentVelocitySlider
        function DescentVelocitySliderValueChanging(app, event)
            changingValue = event.Value;
            app.valV.Text = num2str(changingValue, '%.2f'); 
            app.update_sim();
            
        end

        % Value changing function: FlareHeightSlider
        function FlareHeightSliderValueChanging(app, event)
            changingValue = event.Value;
            app.valFlare.Text = num2str(changingValue, '%.1f'); 
            app.update_sim();
            
        end

        % Value changing function: TrueWSlider
        function TrueWSliderValueChanging(app, event)
            changingValue = event.Value;
            app.valTrueW.Text = num2str(changingValue, '%.2f'); 
            app.update_sim();
        end

        % Value changing function: TrueRSlider
        function TrueRSliderValueChanging(app, event)
            changingValue = event.Value;
            app.valTrueR.Text = num2str(changingValue, '%.3f'); 
            app.update_sim();
            
        end

        % Value changing function: FilterQSlider
        function FilterQSliderValueChanging(app, event)
            changingValue = event.Value;
            app.valFilterQ.Text = num2str(changingValue, '%.2f'); 
            app.update_sim();
            
        end

        % Value changing function: FilterRSlider
        function FilterRSliderValueChanging(app, event)
            changingValue = event.Value;
            app.valFilterR.Text = num2str(changingValue, '%.3f'); 
            app.update_sim();
        end

        % Value changed function: cbFreeFall
        function cbFreeFallValueChanged(app, event)
            app.update_sim();
            
        end

        % Value changed function: cbOutage
        function cbOutageValueChanged(app, event)
            app.update_sim();
            
        end

        % Value changed function: cbSpikes
        function cbSpikesValueChanged(app, event)
            app.update_sim();
            
        end

        % Value changed function: cbMeas
        function cbMeasValueChanged(app, event)
            app.update_sim();
            
        end

        % Value changed function: cbCV
        function cbCVValueChanged(app, event)
            app.update_sim();
            
        end

        % Value changed function: cbCA
        function cbCAValueChanged(app, event)
            app.update_sim();
            
        end

        % Button pushed function: ResetButton
        function ResetButtonPushed(app, event)
            % reset sliders and labels to default values
            app.InitialAltitudeSlider.Value = 4.0; app.valH.Text = '4.0';
            app.DescentVelocitySlider.Value = 1.2; app.valV.Text = '1.20';
            app.FlareHeightSlider.Value = 1.5; app.valFlare.Text = '1.5';
            app.TrueWSlider.Value = 0.5; app.valTrueW.Text = '0.50';
            app.TrueRSlider.Value = 0.01; app.valTrueR.Text = '0.010'; 
            app.FilterQSlider.Value = 1.0; app.valFilterQ.Text = '1.00'; 
            app.FilterRSlider.Value = 0.05; app.valFilterR.Text = '0.050';
        
            % reset checkboxes
            app.cbMeas.Value = 1; app.cbCV.Value = 1; app.cbCA.Value = 1; app.cbSpikes.Value = 1;
            app.cbFreeFall.Value = 0; app.cbOutage.Value = 0;
        
            % re-run simulation
            app.update_sim();
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1250 830];
            app.UIFigure.Name = 'MATLAB App';

            % Create AltitudeAxes
            app.AltitudeAxes = uiaxes(app.UIFigure);
            title(app.AltitudeAxes, 'Title')
            xlabel(app.AltitudeAxes, 'X')
            ylabel(app.AltitudeAxes, 'Y')
            zlabel(app.AltitudeAxes, 'Z')
            app.AltitudeAxes.Position = [40 330 550 290];

            % Create VelocityAxes
            app.VelocityAxes = uiaxes(app.UIFigure);
            title(app.VelocityAxes, 'Title')
            xlabel(app.VelocityAxes, 'X')
            ylabel(app.VelocityAxes, 'Y')
            zlabel(app.VelocityAxes, 'Z')
            app.VelocityAxes.Position = [40 20 550 290];

            % Create AltErrorAxes
            app.AltErrorAxes = uiaxes(app.UIFigure);
            title(app.AltErrorAxes, 'Title')
            xlabel(app.AltErrorAxes, 'X')
            ylabel(app.AltErrorAxes, 'Y')
            zlabel(app.AltErrorAxes, 'Z')
            app.AltErrorAxes.Position = [650 330 550 290];

            % Create VelErrorAxes
            app.VelErrorAxes = uiaxes(app.UIFigure);
            title(app.VelErrorAxes, 'Title')
            xlabel(app.VelErrorAxes, 'X')
            ylabel(app.VelErrorAxes, 'Y')
            zlabel(app.VelErrorAxes, 'Z')
            app.VelErrorAxes.Position = [650 20 550 290];

            % Create SimulationControlDashboardPanel
            app.SimulationControlDashboardPanel = uipanel(app.UIFigure);
            app.SimulationControlDashboardPanel.Title = 'Simulation Control Dashboard';
            app.SimulationControlDashboardPanel.FontWeight = 'bold';
            app.SimulationControlDashboardPanel.FontSize = 13;
            app.SimulationControlDashboardPanel.Position = [40 640 1170 190];

            % Create FlightDynamicsLabel
            app.FlightDynamicsLabel = uilabel(app.SimulationControlDashboardPanel);
            app.FlightDynamicsLabel.FontSize = 13;
            app.FlightDynamicsLabel.FontWeight = 'bold';
            app.FlightDynamicsLabel.Position = [10 140 150 20];
            app.FlightDynamicsLabel.Text = '1. Flight Dynamics';

            % Create InitialAltitudemLabel
            app.InitialAltitudemLabel = uilabel(app.SimulationControlDashboardPanel);
            app.InitialAltitudemLabel.Position = [10 100 120 20];
            app.InitialAltitudemLabel.Text = 'Initial Altitude [m]';

            % Create InitialAltitudeSlider
            app.InitialAltitudeSlider = uislider(app.SimulationControlDashboardPanel);
            app.InitialAltitudeSlider.Limits = [1 8];
            app.InitialAltitudeSlider.ValueChangingFcn = createCallbackFcn(app, @InitialAltitudeSliderValueChanging, true);
            app.InitialAltitudeSlider.Position = [130 110 100 3];
            app.InitialAltitudeSlider.Value = 4;

            % Create valH
            app.valH = uilabel(app.SimulationControlDashboardPanel);
            app.valH.FontWeight = 'bold';
            app.valH.Position = [240 100 40 20];
            app.valH.Text = '4.0';

            % Create DescentVelocitymsLabel
            app.DescentVelocitymsLabel = uilabel(app.SimulationControlDashboardPanel);
            app.DescentVelocitymsLabel.Position = [10 60 130 20];
            app.DescentVelocitymsLabel.Text = 'Descent Velocity [m/s]';

            % Create valV
            app.valV = uilabel(app.SimulationControlDashboardPanel);
            app.valV.FontWeight = 'bold';
            app.valV.Position = [240 60 40 20];
            app.valV.Text = '1.20';

            % Create DescentVelocitySlider
            app.DescentVelocitySlider = uislider(app.SimulationControlDashboardPanel);
            app.DescentVelocitySlider.Limits = [0.1 5];
            app.DescentVelocitySlider.ValueChangingFcn = createCallbackFcn(app, @DescentVelocitySliderValueChanging, true);
            app.DescentVelocitySlider.Position = [130 70 100 3];
            app.DescentVelocitySlider.Value = 1.2;

            % Create FlareHeightSlider
            app.FlareHeightSlider = uislider(app.SimulationControlDashboardPanel);
            app.FlareHeightSlider.Limits = [0.1 8];
            app.FlareHeightSlider.ValueChangingFcn = createCallbackFcn(app, @FlareHeightSliderValueChanging, true);
            app.FlareHeightSlider.Position = [130 30 100 3];
            app.FlareHeightSlider.Value = 1.5;

            % Create FlareHeightmLabel
            app.FlareHeightmLabel = uilabel(app.SimulationControlDashboardPanel);
            app.FlareHeightmLabel.Position = [10 20 130 20];
            app.FlareHeightmLabel.Text = 'Flare Height [m]';

            % Create valFlare
            app.valFlare = uilabel(app.SimulationControlDashboardPanel);
            app.valFlare.FontWeight = 'bold';
            app.valFlare.Position = [240 20 40 20];
            app.valFlare.Text = '1.5';

            % Create EnvironmentGroundTruthLabel
            app.EnvironmentGroundTruthLabel = uilabel(app.SimulationControlDashboardPanel);
            app.EnvironmentGroundTruthLabel.FontSize = 13;
            app.EnvironmentGroundTruthLabel.FontWeight = 'bold';
            app.EnvironmentGroundTruthLabel.FontColor = [0.702 0.102 0.102];
            app.EnvironmentGroundTruthLabel.Position = [300 138 230 22];
            app.EnvironmentGroundTruthLabel.Text = '2. Environment (Ground Truth)';

            % Create EnvDisturbancessigma_wms2Label
            app.EnvDisturbancessigma_wms2Label = uilabel(app.SimulationControlDashboardPanel);
            app.EnvDisturbancessigma_wms2Label.Interpreter = 'tex';
            app.EnvDisturbancessigma_wms2Label.Position = [300 96 170 22];
            app.EnvDisturbancessigma_wms2Label.Text = 'Env. Disturbances(\sigma_w)[m/s^2]';

            % Create valTrueW
            app.valTrueW = uilabel(app.SimulationControlDashboardPanel);
            app.valTrueW.FontWeight = 'bold';
            app.valTrueW.Position = [590 100 40 20];
            app.valTrueW.Text = '0.50';

            % Create TrueWSlider
            app.TrueWSlider = uislider(app.SimulationControlDashboardPanel);
            app.TrueWSlider.Limits = [0 3];
            app.TrueWSlider.ValueChangingFcn = createCallbackFcn(app, @TrueWSliderValueChanging, true);
            app.TrueWSlider.Position = [470 110 110 3];
            app.TrueWSlider.Value = 0.5;

            % Create SensorNoiseRm2Label
            app.SensorNoiseRm2Label = uilabel(app.SimulationControlDashboardPanel);
            app.SensorNoiseRm2Label.Interpreter = 'tex';
            app.SensorNoiseRm2Label.Position = [300 58 139 22];
            app.SensorNoiseRm2Label.Text = 'Sensor Noise (R)[m^2]';

            % Create valTrueR
            app.valTrueR = uilabel(app.SimulationControlDashboardPanel);
            app.valTrueR.FontWeight = 'bold';
            app.valTrueR.Position = [590 60 40 20];
            app.valTrueR.Text = '0.010';

            % Create TrueRSlider
            app.TrueRSlider = uislider(app.SimulationControlDashboardPanel);
            app.TrueRSlider.Limits = [0.001 0.5];
            app.TrueRSlider.ValueChangingFcn = createCallbackFcn(app, @TrueRSliderValueChanging, true);
            app.TrueRSlider.Position = [470 70 110 3];
            app.TrueRSlider.Value = 0.01;

            % Create KalmanFilterAlgorithmLabel
            app.KalmanFilterAlgorithmLabel = uilabel(app.SimulationControlDashboardPanel);
            app.KalmanFilterAlgorithmLabel.FontSize = 13;
            app.KalmanFilterAlgorithmLabel.FontWeight = 'bold';
            app.KalmanFilterAlgorithmLabel.FontColor = [0 0.302 0.702];
            app.KalmanFilterAlgorithmLabel.Position = [660 140 200 20];
            app.KalmanFilterAlgorithmLabel.Text = '3. Kalman Filter (Algorithm)';

            % Create ProcessNoisesigma_ams2Label
            app.ProcessNoisesigma_ams2Label = uilabel(app.SimulationControlDashboardPanel);
            app.ProcessNoisesigma_ams2Label.Interpreter = 'tex';
            app.ProcessNoisesigma_ams2Label.Position = [660 98 152 22];
            app.ProcessNoisesigma_ams2Label.Text = 'Process Noise (\sigma_a)[m/s^2]';

            % Create FilterQSlider
            app.FilterQSlider = uislider(app.SimulationControlDashboardPanel);
            app.FilterQSlider.Limits = [0.01 5];
            app.FilterQSlider.ValueChangingFcn = createCallbackFcn(app, @FilterQSliderValueChanging, true);
            app.FilterQSlider.Position = [830 110 100 3];
            app.FilterQSlider.Value = 1;

            % Create valFilterQ
            app.valFilterQ = uilabel(app.SimulationControlDashboardPanel);
            app.valFilterQ.FontWeight = 'bold';
            app.valFilterQ.Position = [940 100 40 20];
            app.valFilterQ.Text = '1.00';

            % Create MeasCovarianceRm2Label
            app.MeasCovarianceRm2Label = uilabel(app.SimulationControlDashboardPanel);
            app.MeasCovarianceRm2Label.Interpreter = 'tex';
            app.MeasCovarianceRm2Label.Position = [660 60 160 25];
            app.MeasCovarianceRm2Label.Text = 'Meas. Covariance (R)[m^2]';

            % Create FilterRSlider
            app.FilterRSlider = uislider(app.SimulationControlDashboardPanel);
            app.FilterRSlider.Limits = [0.001 0.5];
            app.FilterRSlider.ValueChangingFcn = createCallbackFcn(app, @FilterRSliderValueChanging, true);
            app.FilterRSlider.Position = [830 70 100 3];
            app.FilterRSlider.Value = 0.05;

            % Create valFilterR
            app.valFilterR = uilabel(app.SimulationControlDashboardPanel);
            app.valFilterR.FontWeight = 'bold';
            app.valFilterR.Position = [940 60 40 20];
            app.valFilterR.Text = '0.050';

            % Create ResetButton
            app.ResetButton = uibutton(app.SimulationControlDashboardPanel, 'push');
            app.ResetButton.ButtonPushedFcn = createCallbackFcn(app, @ResetButtonPushed, true);
            app.ResetButton.BackgroundColor = [0.902 0.902 0.902];
            app.ResetButton.Position = [990 140 150 25];
            app.ResetButton.Text = 'Reset Simulation';

            % Create cbFreeFall
            app.cbFreeFall = uicheckbox(app.SimulationControlDashboardPanel);
            app.cbFreeFall.ValueChangedFcn = createCallbackFcn(app, @cbFreeFallValueChanged, true);
            app.cbFreeFall.Text = 'Free Fall Drop (-9.81 m/s²)';
            app.cbFreeFall.FontWeight = 'bold';
            app.cbFreeFall.Position = [990 115 180 20];

            % Create cbOutage
            app.cbOutage = uicheckbox(app.SimulationControlDashboardPanel);
            app.cbOutage.ValueChangedFcn = createCallbackFcn(app, @cbOutageValueChanged, true);
            app.cbOutage.Text = 'Sensor Outage (1 sec)';
            app.cbOutage.FontWeight = 'bold';
            app.cbOutage.FontColor = [0.4 0 0.6];
            app.cbOutage.Position = [990 95 160 20];

            % Create cbSpikes
            app.cbSpikes = uicheckbox(app.SimulationControlDashboardPanel);
            app.cbSpikes.ValueChangedFcn = createCallbackFcn(app, @cbSpikesValueChanged, true);
            app.cbSpikes.Text = 'Inject Spikes';
            app.cbSpikes.FontWeight = 'bold';
            app.cbSpikes.FontColor = [1 0 0];
            app.cbSpikes.Position = [990 75 150 20];
            app.cbSpikes.Value = true;

            % Create cbMeas
            app.cbMeas = uicheckbox(app.SimulationControlDashboardPanel);
            app.cbMeas.ValueChangedFcn = createCallbackFcn(app, @cbMeasValueChanged, true);
            app.cbMeas.Text = 'Show Measurements';
            app.cbMeas.FontWeight = 'bold';
            app.cbMeas.FontColor = [0.8 0.4 0];
            app.cbMeas.Position = [990 50 150 20];
            app.cbMeas.Value = true;

            % Create cbCV
            app.cbCV = uicheckbox(app.SimulationControlDashboardPanel);
            app.cbCV.ValueChangedFcn = createCallbackFcn(app, @cbCVValueChanged, true);
            app.cbCV.Text = 'Show CV Model';
            app.cbCV.FontColor = [0 0 1];
            app.cbCV.Position = [990 28 120 22];
            app.cbCV.Value = true;

            % Create cbCA
            app.cbCA = uicheckbox(app.SimulationControlDashboardPanel);
            app.cbCA.ValueChangedFcn = createCallbackFcn(app, @cbCAValueChanged, true);
            app.cbCA.Text = 'Show CA Model';
            app.cbCA.FontWeight = 'bold';
            app.cbCA.FontColor = [1 0 1];
            app.cbCA.Position = [990 10 120 20];
            app.cbCA.Value = true;

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = kalman_filter_model_comparison_gui_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end