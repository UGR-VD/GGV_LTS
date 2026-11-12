function pt = vehicle_powertrain(veh)
% VEHICLE_POWERTRAIN  HV powertrain torque curve and straight-line acceleration sim.
% Follows the 'HV Powertrain' and 'Acceleration' sheets of UGR27_Vehicle_Dynamics_Spreadsheet.
% Only the HV powertrain is modelled (the active one in the sheet: rear HV motor, no front motor).
%
% With veh.torque_override = true the motor torque comes from veh.torque_override_curve
% ([rpm, Nm], motor output torque) instead, and the curve's last rpm sets the rev limit.
%
% Returns PT with:
%   .motor_rpm, .motor_torque, .wheel_torque      motor table vs motor rpm (0:100:max rpm)
%   .v_grid, .wheel_torque_v, .gear_v, .rpm_v      wheel torque vs speed (0:0.1:50 m/s)
%   .v_top                                          gearing top speed (m/s)
%   .t, .v, .a, .x                                  acceleration sim (time, speed, accel, distance)
%   .t_accel                                        time to cover the acceleration length (x_accel_length)

g = 9.81;
if ~strcmp(veh.powertrain_rear, "HV Powertrain") || ~strcmp(veh.powertrain_front, "None")
    error('vehicle_powertrain:notModelled', ...
        'Only a rear "HV Powertrain" with front "None" is modelled (as set in the spreadsheet).');
end

%% Motor and accumulator ('HV Powertrain')
motor = lookupByName(veh.motors, veh.motor);
cell = lookupByName(veh.cells, veh.cell_type);

V_pack_max = cell.V_max * veh.n_cells_series * veh.n_segments * veh.eff_accumulator * veh.eff_inverter; % V (F5)
I_burst = min(cell.I_burst * veh.n_cells_parallel, veh.I_burst_limit);                                   % A (F7)

nGears = numel(unique(veh.r_gears));
if nGears == 1, eff_gears = 1; else, eff_gears = veh.eff_gears; end     % R8
eff_drivetrain = veh.eff_gearbox * veh.eff_other_drivetrain;           % R11
r_eff = veh.r_gears * veh.r_planetary;                                 % N11:N14

% Motor table: 0:100:max rpm (U3:U63)
rpm = (0:100:motor.rpm_max)';
n = numel(rpm);
I = NaN(n, 1);
T_motor = NaN(n, 1);
for i = 1:n
    w_e = motor.pole_pairs * 2 * pi * rpm(i) / 60;                    % electrical speed (V)
    if i == 1
        V = V_pack_max;                                               % W3
    else
        V = V_pack_max - veh.R_internal * I(i - 1);                   % W4 = F5 - J21*AD3 (voltage sag)
    end
    X = w_e^2 * motor.L_q^2 + motor.R_stator^2;
    Y = 2 * motor.R_stator * w_e * motor.psi;
    Z = w_e^2 * motor.psi^2 - (V / sqrt(3))^2;
    disc = Y^2 - 4 * X * Z;
    if disc < 0 || isnan(disc)
        I_motor = NaN;
    else
        I_motor = -Y + sqrt(disc) / (2 * X);   % as written in the sheet: -Y + SQRT(...)/(2X)
    end
    I_acc = veh.P_accumulator_max / V * 1000;                         % AC
    Is = [I_motor, I_burst, I_acc];
    if any(isnan(Is)), I(i) = NaN; else, I(i) = min(Is); end           % AD (MIN propagates #N/A)
    T = 1.5 * I(i) * motor.psi * motor.pole_pairs;                 % NaN stays NaN (Excel MIN with #N/A)
    if ~isnan(T), T = min(motor.T_max, T); end
    if T < 0 || isnan(T), T_motor(i) = NaN; else, T_motor(i) = veh.eff_motor * T; end  % AE
end
rpm_max = motor.rpm_max;
pt.override = isfield(veh, 'torque_override') && veh.torque_override;
if pt.override
    % User-supplied motor torque curve replaces the motor / accumulator model
    [rpm, T_motor] = overrideCurve(veh.torque_override_curve);
    rpm_max = rpm(end);
    I = NaN(size(rpm));
end
% Wheel torque for each gear (AH, AJ, AL, AN)
wheel_torque = veh.r_final_drive * T_motor * eff_gears * veh.r_gears;   % n x 4

pt.motor_rpm = rpm;
pt.motor_current = I;
pt.motor_torque = T_motor;
pt.wheel_torque = wheel_torque;

%% Wheel torque vs speed (AQ:BG)
v_grid = (0:0.1:50)';
m = numel(v_grid);
nG = numel(veh.r_gears);
rpm_g = NaN(m, nG); T_g = NaN(m, nG); P_g = NaN(m, nG);
for k = 1:nG
    r = r_eff(k) * v_grid / (veh.x_LoadedRadiusR / 1000) / (2 * pi / 60);
    r(r > rpm_max) = NaN;
    rpm_g(:, k) = r;
    T_g(:, k) = eff_drivetrain * forecastBracket(rpm, wheel_torque(:, k), r);
    P_g(:, k) = round(10 * T_g(:, k) .* r / veh.Nm_to_kW / r_eff(k)) / 10;   % AT = ROUND(T*rpm/J19/N11, 1)
end
gear = NaN(m, 1); T_v = NaN(m, 1); rpm_v = NaN(m, 1);
for j = 1:m
    p = P_g(j, :);
    if all(isnan(p)), continue, end
    best = max(p(~isnan(p)));
    if v_grid(j) ~= 0 && best == 0, continue, end                     % BG = NA when max power is 0
    k = find(p == best, 1);
    gear(j) = k; T_v(j) = T_g(j, k); rpm_v(j) = rpm_g(j, k);
end
pt.v_grid = v_grid;
pt.gear_v = gear;
pt.rpm_v = rpm_v;
pt.wheel_torque_v = T_v;

% Gearing top speed (N23): speed of the last row before the first #N/A
firstNA = find(isnan(T_v), 1);
if isempty(firstNA)
    pt.v_top = max(v_grid);
else
    pt.v_top = v_grid(max(firstNA - 1, 1));
end

%% Acceleration sim ('Acceleration' sheet, 1000 steps)
M = veh.m_total;
mF = veh.m_totalF;
mR = veh.m_totalR;
h = veh.h_total_cog;
WB = veh.x_wheelbase;
dt = veh.dt_accel;
mu_long = @(Fz) (veh.r_mu_long_gradient * Fz + veh.r_mu_long_0) * veh.r_mu_long_scale;
q = @(v) 0.5 * veh.rho_air_accel * v^2;

N = 1000;
t = (0:N - 1)' * dt;
v = zeros(N, 1); a = zeros(N, 1); x = zeros(N, 1);
C = 0;
for i = 1:N
    if i > 1
        v(i) = v(i - 1) + a(i - 1) * dt;
        x(i) = v(i - 1) * dt + 0.5 * a(i - 1) * dt^2 + x(i - 1);
        C = M * a(i - 1) * h / 1000 / (WB / 1000);                     % longitudinal load transfer (N)
    end
    aeroF = q(v(i)) * veh.ClA_accel * veh.r_aero_balance_accel;
    aeroR = q(v(i)) * veh.ClA_accel * (1 - veh.r_aero_balance_accel);
    drag = q(v(i)) * veh.CdA_accel;
    FzF = mF * g / 2 - C / 2 + aeroF / 2;                              % single front tyre
    FzR = mR * g / 2 + C / 2 + aeroR / 2;                              % single rear tyre
    gripR = 2 * FzR * mu_long(FzR);
    rollRes = 2 * (FzF + FzR) * veh.r_rolling_resistance;
    radiusR = veh.x_Tyre_diameterR / 1000 / 2 - FzR / veh.k_TyreR / 1000;
    % Rear wheel torque (single gear: no shift delays). #N/A -> 0
    T_wheel = forecastBracket(v_grid, T_v, v(i));
    if isnan(T_wheel), T_wheel = 0; end
    if nG > 1 && nGears > 1
        % Shift delays (AP10) are not modelled for multi-gear setups
    end
    F_pt = T_wheel / radiusR;
    if v(i) > pt.v_top
        aR = 0;
    else
        aR = min(gripR, F_pt) / M;
    end
    a(i) = aR - (rollRes + drag) / M;                                  % front motor "None" -> 0
end
pt.t = t;
pt.v = v;
pt.a = a;
pt.x = x;
idx = find(x > veh.x_accel_length, 1);
if isempty(idx), pt.t_accel = NaN; else, pt.t_accel = t(idx); end
end


function [rpm, T] = overrideCurve(curve)
% Resample a user [rpm, torque] curve onto a 100 rpm grid from 0 to its last rpm.
% Below the first rpm the first torque is held; rows with NaN are ignored.
if isempty(curve) || size(curve, 2) < 2
    error('vehicle_powertrain:override', 'Torque override is on but no rpm / torque curve has been supplied.');
end
curve = double(curve(:, 1:2));
curve = curve(all(isfinite(curve), 2), :);
curve = sortrows(curve, 1);
if size(curve, 1) < 2
    error('vehicle_powertrain:override', 'The torque override curve needs at least two valid rows.');
end
if any(diff(curve(:, 1)) == 0)
    error('vehicle_powertrain:override', 'The torque override curve has repeated rpm values.');
end
if any(curve(:, 1) < 0) || any(curve(:, 2) < 0)
    error('vehicle_powertrain:override', 'The torque override curve must have rpm and torque >= 0.');
end
rpm = unique([(0:100:curve(end, 1))'; curve(end, 1)]);
T = interp1(curve(:, 1), curve(:, 2), rpm, 'linear');
T(rpm < curve(1, 1)) = curve(1, 2);
end

function item = lookupByName(list, name)
names = cellfun(@char, {list.name}, 'UniformOutput', false);
idx = find(strcmp(names, char(name)), 1);
if isempty(idx)
    error('vehicle_powertrain:unknown', 'Unknown entry "%s".', char(name));
end
item = list(idx);
end

function y = forecastBracket(xs, ys, xq)
% Excel FORECAST(xq, OFFSET(ys, MATCH(xq, xs, 1)-1, 0, 2, 1), OFFSET(xs, ...)):
% straight line through the two table rows at and after the largest xs <= xq.
% Returns NaN where either row is #N/A or the bracket runs off the table.
y = NaN(size(xq));
for k = 1:numel(xq)
    if isnan(xq(k)) || xq(k) < xs(1), continue, end
    i = find(xs <= xq(k), 1, 'last');
    if i >= numel(xs), continue, end
    x1 = xs(i); x2 = xs(i + 1); y1 = ys(i); y2 = ys(i + 1);
    if isnan(y1) || isnan(y2), continue, end
    y(k) = y1 + (y2 - y1) * (xq(k) - x1) / (x2 - x1);
end
end
