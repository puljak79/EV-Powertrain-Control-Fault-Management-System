% =========================================================================
% EV Powertrain Automated Test Suite & Rigorous Verification (run_ev_tests.m)
% =========================================================================
clear; clc;

model_name = 'EV_powertrain_system';
load_system(model_name);

fprintf('\n========================================\n');
fprintf('EV Powertrain Test Suite - Starting...\n');
fprintf('========================================\n');
fprintf('[NOTE] key_on is active and set to 1.\n');

% 1. Interactive user input for speed and simulation time
prompt_speed = 'Enter desired vehicle speed (e.g., 80): ';
desired_speed = input(prompt_speed);

prompt_time = 'Enter simulation time Tsim (Must be at least 60 seconds): ';
Tsim = input(prompt_time);

if Tsim < 60
    error('ERROR: Simulation time (Tsim) must be at least 60 seconds (>= 60) due to braking tests!');
end

prompt_start = 'Enter 1 to activate start button (Mandatory 1 to proceed): ';
start_val = input(prompt_start);

if start_val ~= 1
    error('PROGRAM ABORTED: Start button must be set to 1 to execute the program!');
end

% INITIALIZATION OF ALL VARIABLES
ref_speed     = [0, desired_speed; Tsim, desired_speed];
fault_flag    = [0, 0; Tsim, 0];
road_slope    = [0, 0; Tsim, 0];
key_on        = [0, 1; Tsim, 1];
start_button  = [0, start_val; Tsim, start_val];
brake_pressed = [0, 0; Tsim, 0];

assignin('base', 'ref_speed', ref_speed);
assignin('base', 'fault_flag', fault_flag);
assignin('base', 'road_slope', road_slope);
assignin('base', 'key_on', key_on);
assignin('base', 'start_button', start_button);
assignin('base', 'brake_pressed', brake_pressed);

fprintf('[INFO] All inputs initialized. Running rigorous tests...\n\n');

% -------------------------------------------------------------------------
% TEST 1: Normal Operation (No faults, flat road)
% -------------------------------------------------------------------------
fprintf('Running Test 1: Normal Operation...\n');
out1 = sim(model_name, 'StopTime', num2str(Tsim));

s1_speed = out1.logsout.get('Speed').Values;
fault_code_1 = out1.logsout.get('fault_code').Values.Data(end);
final_speed_1 = s1_speed.Data(end);

if fault_code_1 == 0 && abs(final_speed_1 - desired_speed) < 2
    fprintf('[PASS] Test 1: Normal operation successful (No faults, target speed reached).\n');
else
    fprintf('[FAIL] Test 1: Failed! Fault code: %d, Final speed: %.2f\n', fault_code_1, final_speed_1);
end

% -------------------------------------------------------------------------
% TEST 2: Fault Injection Test
% -------------------------------------------------------------------------
fprintf('\nRunning Test 2: Fault Injection...\n');
assignin('base', 'fault_flag', [0, 1; Tsim, 1]);

out2 = sim(model_name, 'StopTime', num2str(Tsim));

fault_code_2 = out2.logsout.get('fault_code').Values.Data(end);
if fault_code_2 ~= 0
    fprintf('[PASS] Test 2: Fault successfully detected and registered (Code: %d).\n', fault_code_2);
else
    fprintf('[FAIL] Test 2: Fault flag set, but system did not register a fault!\n');
end

% Reset fault_flag to 0
assignin('base', 'fault_flag', [0, 0; Tsim, 0]);

% -------------------------------------------------------------------------
% TEST 3: Uphill Driving Test (8% incline)
% -------------------------------------------------------------------------
fprintf('\nRunning Test 3: Uphill Driving (8%% incline)...\n');
assignin('base', 'road_slope', [0, 8; Tsim, 8]);

out3 = sim(model_name, 'StopTime', num2str(Tsim));
s3_speed = out3.logsout.get('Speed').Values;

% RIGOROUS CHECK: Fail if the vehicle rolls backward (speed < 0)
if any(s3_speed.Data < -0.1)
    fprintf('[FAIL] Test 3: Vehicle rolled backward (negative speed detected on uphill)!\n');
else
    fprintf('[PASS] Test 3: Uphill simulation completed successfully without rolling back.\n');
end

% Reset road slope to 0
assignin('base', 'road_slope', [0, 0; Tsim, 0]);

% -------------------------------------------------------------------------
% TEST 4: Closed-Loop Deceleration Step Response (Speed Tracking at 50s)
% -------------------------------------------------------------------------
fprintf('\nRunning Test 4: Closed-Loop Deceleration Step Response (t = 50s)...\n');

ref_speed_brake1 = [0, desired_speed; 50, desired_speed; 50.01, 0; Tsim, 0];
brake_pressed_b1 = [0, 0; 49.9, 0; 50, 1; Tsim, 1];

assignin('base', 'ref_speed', ref_speed_brake1);
assignin('base', 'brake_pressed', brake_pressed_b1);

out4 = sim(model_name, 'StopTime', num2str(Tsim));

s4_speed = out4.logsout.get('Speed').Values;
idx_stopped1 = find(s4_speed.Time >= 50 & s4_speed.Data <= 0.1, 1);

if ~isempty(idx_stopped1)
    t_stop1 = s4_speed.Time(idx_stopped1);
    response_time1 = t_stop1 - 50;
    fprintf('[PASS] Test 4: Closed-loop tracked stop at t = %.2f s. Response time: %.2f seconds.\n', t_stop1, response_time1);
else
    fprintf('[FAIL] Test 4: Vehicle failed to reach zero speed within the simulation time after braking command.\n');
end

% Reset brake_pressed back to 0
assignin('base', 'brake_pressed', [0, 0; Tsim, 0]);

% -------------------------------------------------------------------------
% TEST 5: Speed Limit Regulation (50 km/h between 45th and 55th second)
% -------------------------------------------------------------------------
fprintf('\nRunning Test 5: Speed limit regulation (50 km/h from 45s to 55s)...\n');

ref_speed_brake2 = [0, desired_speed; 44.9, desired_speed; 45, 50; 55, 50; 55.1, desired_speed; Tsim, desired_speed];
assignin('base', 'ref_speed', ref_speed_brake2);

out5 = sim(model_name, 'StopTime', num2str(Tsim));
s5_speed = out5.logsout.get('Speed').Values;

% RIGOROUS CHECK: Check if average speed in the window is close to 50 km/h
idx_interval = find(s5_speed.Time >= 47 & s5_speed.Time <= 53);
if ~isempty(idx_interval)
    avg_speed_restricted = mean(s5_speed.Data(idx_interval));
    if abs(avg_speed_restricted - 50) < 5
        fprintf('[PASS] Test 5: Speed limit successfully regulated (Avg speed in window: %.2f km/h).\n', avg_speed_restricted);
    else
        fprintf('[FAIL] Test 5: Speed regulation failed to track 50 km/h (Avg speed: %.2f km/h).\n', avg_speed_restricted);
    end
else
    fprintf('[FAIL] Test 5: Speed profile evaluation window empty.\n');
end

% =========================================================================
% INTERACTIVE PLOT SELECTION
% =========================================================================
fprintf('\n----------------------------------------\n');
fprintf('PLOT SELECTION MENU\n');
fprintf('----------------------------------------\n');
fprintf('Available tests:\n');
fprintf('  1 = Normal Operation (Test 1)\n');
fprintf('  2 = Fault Injection (Test 2)\n');
fprintf('  3 = Uphill Driving (Test 3)\n');
fprintf('  4 = Closed-Loop Deceleration Step Response (Test 4)\n');
fprintf('  5 = Speed Limit Regulation 50kmh (Test 5)\n');
fprintf('----------------------------------------\n');

plot_input = input('Enter test numbers to plot as a vector (e.g., [1, 3, 4, 5]), or type "all": ', 's');

if strcmpi(plot_input, 'all') || isempty(plot_input)
    selected_tests = [1, 2, 3, 4, 5];
else
    selected_tests = str2num(plot_input); %#ok<ST2NM>
end

fprintf('\nGenerating selected plots with dark mode (black background)...\n');
opengl software;

% --- PLOT TEST 1 ---
if ismember(1, selected_tests)
    s1_temp  = out1.logsout.get('Temperature').Values;
    s1_curr  = out1.logsout.get('Current').Values;
    st1 = max(1, floor(length(s1_speed.Data) / 5000));

    figure('Name', 'Test 1 - Normal Operation', 'Color', 'k');
    subplot(3,1,1); plot(s1_speed.Time(1:st1:end), s1_speed.Data(1:st1:end), 'cyan', 'LineWidth', 1.5);
    ylabel('Speed', 'Color', 'w'); title('Test 1: Normal Operation', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    subplot(3,1,2); plot(s1_temp.Time(1:st1:end), s1_temp.Data(1:st1:end), 'red', 'LineWidth', 1.5);
    ylabel('Temp (°C)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    subplot(3,1,3); plot(s1_curr.Time(1:st1:end), s1_curr.Data(1:st1:end), 'green', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Color', 'w'); ylabel('Current (A)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
end

% --- PLOT TEST 2 ---
if ismember(2, selected_tests)
    s2_speed = out2.logsout.get('Speed').Values;
    s2_temp  = out2.logsout.get('Temperature').Values;
    s2_curr  = out2.logsout.get('Current').Values;
    st2 = max(1, floor(length(s2_speed.Data) / 5000));

    figure('Name', 'Test 2 - Fault Injection', 'Color', 'k');
    subplot(3,1,1); plot(s2_speed.Time(1:st2:end), s2_speed.Data(1:st2:end), 'cyan', 'LineWidth', 1.5);
    ylabel('Speed', 'Color', 'w'); title('Test 2: Fault Injection', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    subplot(3,1,2); plot(s2_temp.Time(1:st2:end), s2_temp.Data(1:st2:end), 'red', 'LineWidth', 1.5);
    ylabel('Temp (°C)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    subplot(3,1,3); plot(s2_curr.Time(1:st2:end), s2_curr.Data(1:st2:end), 'green', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Color', 'w'); ylabel('Current (A)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
end

% --- PLOT TEST 3 ---
if ismember(3, selected_tests)
    s3_temp  = out3.logsout.get('Temperature').Values;
    s3_curr  = out3.logsout.get('Current').Values;
    st3 = max(1, floor(length(s3_speed.Data) / 5000));

    figure('Name', 'Test 3 - Uphill Driving', 'Color', 'k');
    subplot(3,1,1); plot(s3_speed.Time(1:st3:end), s3_speed.Data(1:st3:end), 'cyan', 'LineWidth', 1.5);
    ylabel('Speed', 'Color', 'w'); title('Test 3: Uphill Driving (8% Incline)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    subplot(3,1,2); plot(s3_temp.Time(1:st3:end), s3_temp.Data(1:st3:end), 'red', 'LineWidth', 1.5);
    ylabel('Temp (°C)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    subplot(3,1,3); plot(s3_curr.Time(1:st3:end), s3_curr.Data(1:st3:end), 'green', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Color', 'w'); ylabel('Current (A)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
end

% --- PLOT TEST 4 ---
if ismember(4, selected_tests)
    s4_temp  = out4.logsout.get('Temperature').Values;
    s4_curr  = out4.logsout.get('Current').Values;
    st4 = max(1, floor(length(s4_speed.Data) / 5000));

    figure('Name', 'Test 4 - Closed-Loop Deceleration Step Response', 'Color', 'k');
    subplot(3,1,1); plot(s4_speed.Time(1:st4:end), s4_speed.Data(1:st4:end), 'cyan', 'LineWidth', 1.5);
    ylabel('Speed', 'Color', 'w'); title('Test 4: Closed-Loop Deceleration Step Response', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    subplot(3,1,2); plot(s4_temp.Time(1:st4:end), s4_temp.Data(1:st4:end), 'red', 'LineWidth', 1.5);
    ylabel('Temp (°C)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    subplot(3,1,3); plot(s4_curr.Time(1:st4:end), s4_curr.Data(1:st4:end), 'green', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Color', 'w'); ylabel('Current (A)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
end

% --- PLOT TEST 5 ---
if ismember(5, selected_tests)
    s5_temp  = out5.logsout.get('Temperature').Values;
    s5_curr  = out5.logsout.get('Current').Values;
    st5 = max(1, floor(length(s5_speed.Data) / 5000));

    figure('Name', 'Test 5 - Speed Limit Regulation (45s-55s)', 'Color', 'k');
    subplot(3,1,1); plot(s5_speed.Time(1:st5:end), s5_speed.Data(1:st5:end), 'cyan', 'LineWidth', 1.5);
    ylabel('Speed', 'Color', 'w'); title('Test 5: Speed Limit Regulation (50 km/h from 45s to 55s)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    subplot(3,1,2); plot(s5_temp.Time(1:st5:end), s5_temp.Data(1:st5:end), 'red', 'LineWidth', 1.5);
    ylabel('Temp (°C)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    subplot(3,1,3); plot(s5_curr.Time(1:st5:end), s5_curr.Data(1:st5:end), 'green', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Color', 'w'); ylabel('Current (A)', 'Color', 'w'); grid on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
end

fprintf('\nSelected test execution and plotting completed successfully!\n');