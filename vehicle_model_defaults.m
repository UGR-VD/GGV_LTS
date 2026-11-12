function inp = vehicle_model_defaults()
% VEHICLE_MODEL_DEFAULTS  Input values for the vehicle model.
% Only the cells marked as inputs (orange) in UGR27_Vehicle_Dynamics_Spreadsheet are here.
% Everything else is calculated in vehicle_model_calc.m.
%
% Naming: first letters give the value type
% m=mass, M=Moment, r=ratio, h=height, x=displacement, k=stiffness, theta=angle etc..
% Engineering units (mm, kg, N)

%% General Parameters

% Mass
inp.m_vehicle = 231.5;                 % kg
inp.m_driver = 68;                     % kg
inp.r_vehicle_front_rear_mass = 0.43;  % - (front fraction)
inp.h_vehicle_cog = 269.1;             % mm
inp.h_driver_cog = 330;                % mm
inp.r_driver_front_rear_mass = 0.7;    % - (front fraction)

% Unsprung mass
inp.m_unsprungF = 13;                  % kg
inp.m_unsprungR = 12.4;                % kg

% Dimensions
inp.x_wheelbase = 1525;                % mm
inp.x_trackF = 1190;                   % mm
inp.x_trackR = 1150;                   % mm

% Tyres
inp.k_TyreF_imperial = 570;            % lb/in
inp.k_TyreR_imperial = 570;            % lb/in
inp.x_Tyre_diameterF_imperial = 16.2;  % in
inp.x_Tyre_diameterR_imperial = 16.2;  % in
inp.x_Tyre_widthF = 205;               % mm
inp.x_Tyre_widthR = 205;               % mm
inp.x_rim_offset = 22;                 % mm

%% Springs

inp.k_springF_imperial = 300;          % lb/in
inp.k_springR_imperial = 250;          % lb/in
inp.r_MR_lowF = 0.99;                  % -
inp.r_MR_highF = 1.13;                 % -
inp.MR_settingF = "High";              % "High" or "Low"
inp.r_MR_lowR = 0.88;                  % -
inp.r_MR_highR = 1.09;                 % -
inp.MR_settingR = "Low";               % "High" or "Low"

%% Dampers

inp.x_shock_extended = 200;            % mm, fully extended length
inp.x_shock_compressed = 143;          % mm, fully compressed length

%% Brakes

inp.r_brake_splitF = 0.74;             % - (rear = 1 - front)
inp.brake_typeR = "Outboard";          % "Outboard" or "Inboard"

%% Roll (not from the spreadsheet)

inp.k_ARBF = 0;                        % Nm/deg
inp.k_ARBR = 0;                        % Nm/deg

%% Anti-roll bars ('Springs' sheet, Roll Springs)
% Setting "Soft"/"Medium"/"Hard" picks the moment arm; "Detached" = no bar
inp.ARB_settingF = "Detached";         inp.ARB_settingR = "Detached";
inp.r_ARB_MR_lowF = 0;                 inp.r_ARB_MR_lowR = 0.32;     % - (front blank in sheet = 0)
inp.r_ARB_MR_highF = 0;                inp.r_ARB_MR_highR = 0.4;     % -
inp.x_ARB_arm_softF = 110;             inp.x_ARB_arm_softR = 85.4399740857855;   % mm
inp.x_ARB_arm_mediumF = 100;           inp.x_ARB_arm_mediumR = 60.41520137402254; % mm
inp.x_ARB_arm_hardF = 90;              inp.x_ARB_arm_hardR = 48.92;  % mm
inp.ARB_materialF = "T45";             inp.ARB_materialR = "T45";
inp.x_ARB_ODF = 12.7;                  inp.x_ARB_ODR = 12.7;         % mm
inp.x_ARB_IDF = 10.92;                 inp.x_ARB_IDR = 10.9;         % mm

%% Heave springs ('Springs' sheet)
% Setting "Soft"/"Medium"/"Hard" picks the motion ratio; "Detached" = no heave spring
inp.heave_settingF = "Detached";       inp.heave_settingR = "Detached";
inp.r_heave_MR_soft_lowF = 0.27;       inp.r_heave_MR_soft_highF = 0.31;   % - (front only in sheet)
inp.r_heave_MR_medium_lowF = 0.34;     inp.r_heave_MR_medium_highF = 0.39; % -
inp.r_heave_MR_hard_lowF = 0.41;       inp.r_heave_MR_hard_highF = 0.47;   % -
inp.heave_materialF = "T45";           inp.heave_materialR = "4130";
inp.x_heave_lengthF = 640.86;          inp.x_heave_lengthR = 0;      % mm
inp.x_heave_armF = 53;                 inp.x_heave_armR = 0;         % mm
inp.x_heave_ODF = 12.7;                inp.x_heave_ODR = 12.7;       % mm
inp.x_heave_IDF = 10.9;                inp.x_heave_IDR = 10.9;       % mm

%% Tyre model ('Tyres & Load Transfer' sheet)
% Friction coefficient = (gradient * Fz + mu0) * scaling, Fz = single tyre load (N)
inp.r_mu_lat_gradient = -0.0002197802197802193;  % 1/N
inp.r_mu_lat_0 = 2.622417582417582;              % -
inp.r_mu_lat_scale = 0.68;                       % -
inp.r_mu_long_gradient = -5e-05;                 % 1/N
inp.r_mu_long_0 = 2.62;                          % -
inp.r_mu_long_scale = 0.68;                      % -
inp.drive_type = "RWD";                          % "RWD" or "AWD"

%% Aero ('Tyres & Load Transfer' + 'Aeromap' sheets)
inp.h_static_FRH = 30;                 % mm, static front ride height
inp.h_static_RRH = 30;                 % mm, static rear ride height
inp.rho_air = 1.2;                     % kg/m^3 (load transfer sheet)
inp.x_aero_area = 1;                   % m^2 (Cl values are ClA)
% Cl = c0 + c1*f + c2*r + c3*f*r + c4*f^2 + c5*r^2 + c6*f^3 + c7*r^3 + c8*f^2*r + c9*f*r^2  (f = FRH, r = RRH in mm)
inp.aeromap_ClF = [1.008, 0.05171, 0.0668, -0.0003812, -0.002372, -0.001723, 2.753e-05, 1.106e-05, -1.143e-05, 1.981e-05];
inp.aeromap_ClR = [1.123, 0.04735, 0.0579, -0.00158, -0.0004684, -0.001606, -2.151e-06, 1.763e-05, 2.093e-05, 2.973e-06];

%% Skid pad
inp.x_skidpad_radius = 8.625;          % m
inp.v_skidpad = NaN;                   % m/s, NaN = solve v = sqrt(aLat*g*R); or type a speed (sheet has 12.2074)

%% HV Powertrain ('HV Powertrain' sheet)
inp.cell_type = "VTC6";                % "VTC6" or "P50B"
inp.n_cells_series = 12;               % - per segment
inp.n_cells_parallel = 9;              % -
inp.n_segments = 5;                    % -
inp.I_burst_limit = 214;               % A, max burst current (limited)
inp.motor = "208 MV";                  % Emrax motor (see motor table below)
inp.R_internal = 0.12;                 % Ohm, accumulator internal resistance
inp.P_accumulator_max = 80;            % kW
inp.r_planetary = 4.37;                % -, planetary gearbox ratio
inp.r_final_drive = 4.37;              % -
inp.r_gears = [1.133, 1.133, 1.133, 1.133]; % - (make all equal for 1 gear; descending order)
inp.eff_accumulator = 1;               % -
inp.eff_inverter = 0.92;               % -
inp.eff_motor = 0.91;                  % -
inp.eff_gears = 0.98;                  % - (ignored with 1 gear)
inp.eff_gearbox = 0.97;                % -
inp.eff_other_drivetrain = 0.957;      % -
inp.Nm_to_kW = 9548.8;                 % rpm conversion

%% Acceleration simulation ('Acceleration' sheet)
inp.powertrain_front = "None";         % "None" or "HV Powertrain" (only HV is modelled)
inp.powertrain_rear = "HV Powertrain";
inp.dt_accel = 0.02;                   % s, time step
inp.x_accel_length = 75;               % m, acceleration event length
inp.r_rolling_resistance = 0.02;       % -
inp.t_shift_delay_front = 0;           % s
inp.t_shift_delay_rear = 0.2;          % s

%% Spreadsheet constants that are not marked as inputs (kept here so they can be changed)
inp.g_lat_top_speed_load_transfer = 2.05; % g, fixed lateral g for the top-speed load transfer block
inp.r_roll_correctionF = 1;            % -, front roll stiffness correction factor ('Roll')
inp.r_roll_correctionR = 1;            % -, rear
inp.r_aero_correctionF = 1;            % -, Aeromap ClF correction factor
inp.r_aero_correctionR = 1;            % -, Aeromap ClR correction factor
inp.ClA_accel = 3.22;                  % -, total ClA used by the acceleration sim
inp.r_aero_balance_accel = 0.467;      % -, front aero balance used by the acceleration sim
inp.CdA_accel = 1.47;                  % -, CdA used by the acceleration sim
inp.rho_air_accel = 1.225;             % kg/m^3, used by the acceleration sim

% Material table ('Springs' sheet): name, UTS (MPa), shear modulus G (GPa), E (GPa), density (kg/m^3)
inp.materials = struct('name', {"T45", "4130", "Ti-6Al-4V", "7075-T6"}, ...
    'UTS', {620, 517, 975, 524}, 'G', {80, 80, 44, 26.9}, 'E', {200, 210, 110, 71.7}, 'rho', {7850, 7850, 4430, 2810});

% Emrax motor table ('HV Powertrain' sheet)
inp.motors = struct( ...
    'name',  {"188 HV", "188 MV", "188 LV", "208 HV", "208 MV", "208 LV", "228 HV", "228 MV", "228 LV"}, ...
    'R_stator', {0.0012, 0.0005, 8e-05, 0.012, 0.0057, 0.0008, 0.0018, 0.0008, 0.000112}, ... % Ohm
    'L_q',   {0.000102, 4.4e-05, 6e-06, 0.00013, 5.6e-05, 7.5e-06, 0.000183, 0.00079, 1.06e-05}, ... % H
    'psi',   {0.033, 0.022, 0.008, 0.0393, 0.0257, 0.0095, 0.0542, 0.0355, 0.0131}, ...           % Wb, back EMF constant
    'pole_pairs', {10, 10, 10, 10, 10, 10, 10, 10, 10}, ...
    'T_max', {90, 90, 90, 150, 150, 150, 240, 240, 240}, ...                                     % Nm
    'rpm_max', {6500, 6500, 6500, 6000, 6000, 6000, 5500, 5500, 5500}, ...
    'V_max', {430, 300, 110, 470, 320, 125, 670, 470, 130});                                    % V

% Cell table ('HV Powertrain' sheet)
inp.cells = struct('name', {"VTC6", "P50B"}, 'V_discharged', {2.5, 2.5}, 'V_nominal', {3.6, 3.6}, ...
    'V_max', {4.2, 4.2}, 'I_continuous', {15, 30}, 'I_burst', {30, 60}, 'capacity', {3, 5});

%% Suspension Geometry
% [X, Y, Z], origin at intersection between front axle centreline, ground plane and vehicle centreline
% X = Longitudinal, Y = Lateral (mirrored LHS/RHS), Z = Height. All in mm.
% (Spreadsheet uses SolidWorks axes: each point here is [SW Z, SW X, SW Y].)
% NaN = calculated, not an input (filled in by vehicle_model_calc), or blank in the spreadsheet.

% Front axle
inp.upr_uprightF     = [-18.52, 529.42, 271.68];
inp.lwr_uprightF     = [-0.46, 564, 128.28];
inp.uprfwd_outbrdF   = [120, 285, 183.55];
inp.uprrwd_outbrdF   = [-105, NaN, 183.55];             % Y = forward pickup Y (calculated)
inp.lwrfwd_outbrdF   = [120, 253, 85.13];
inp.lwrrwd_outbrdF   = [-117.5, NaN, 85.13];            % Y = forward pickup Y (calculated)
inp.toe_outbrdF      = [-85.4, 545, 170];
inp.toe_inbrdF       = [-85.4, 115, 81.75];
inp.prod_outbrdF     = [-1, 507.14, 142.6];
inp.prod_inbrdF      = [-1, 348.690515, 365.272035];    % Push/Pullrod to Rocker MR1
inp.prod_inbrd_MR2F  = [-1, 340.65, 359.32];            % Push/Pullrod to Rocker MR2
inp.rocker_centreF   = [-1.00, 292.31, 322.16];
inp.rocker_springF   = [-1.0, 362.278969, 385.645387];
inp.spring_chassisF  = [-1.00, 235.979999, 524.902595];
inp.arb_upr_droplinkF = [NaN, NaN, NaN];                % Z = rocker centre Z (calculated); X, Y blank in sheet
inp.arb_lwr_droplinkF = [NaN, NaN, NaN];                % Y = upper droplink Y (calculated); X, Z blank in sheet
inp.arb_centreF      = [NaN, NaN, NaN];                 % Y = 0, Z = lower droplink Z (calculated); X blank in sheet

% Rear axle
inp.upr_uprightR     = [-1513.02, 502.14, 273.24];
inp.lwr_uprightR     = [-1527.63, 542.14, 123.23];
inp.uprfwd_outbrdR   = [-1278, 293, 168.04];
inp.uprrwd_outbrdR   = [-1494.3, NaN, 168.04];          % Y = forward pickup Y (calculated)
inp.lwrfwd_outbrdR   = [-1278, 232, 67];
inp.lwrrwd_outbrdR   = [-1494.3, NaN, 66];              % Y = forward pickup Y (calculated)
inp.toe_outbrdR      = [-1437, 495.07, 163.8];
inp.toe_inbrdR       = [-1437, 241.2, 94];
inp.prod_outbrdR     = [-1494.1, 450, 279.87];
inp.prod_inbrdR      = [-1494.1, 254.75, 447.48];       % Push/Pullrod to Rocker MR1
inp.prod_inbrd_MR2R  = [-1494.1, 254.5, 447.24];        % Push/Pullrod to Rocker MR2
inp.rocker_centreR   = [-1494.1, 220, 407];
inp.rocker_springR   = [-1494.1, 195.7, 485.22];
inp.spring_chassisR  = [-1494.1, 24.56, 432.06];
inp.arb_upr_droplinkR = [-1494.1, 247, NaN];            % Z = rocker centre Z (calculated)
inp.arb_lwr_droplinkR = [-1494.1, NaN, 321.32];         % Y = upper droplink Y (calculated). Sheet has X = +1494.1 - check
inp.arb_centreR      = [-1566.43, NaN, NaN];            % Y = 0, Z = lower droplink Z (calculated)
end
