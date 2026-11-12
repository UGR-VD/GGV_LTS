function [veh, calcCoords] = vehicle_model_calc(inp)
% VEHICLE_MODEL_CALC  Calculate the vehicle model from its inputs (see vehicle_model_defaults.m).
% Same formulas as UGR27_Vehicle_Dynamics_Spreadsheet ('General Parameters', 'Springs',
% 'Suspension Geometry'). Returns VEH: every input plus every calculated value.
% CALCCOORDS lists the hardpoint coordinates that are calculated rather than input
% (field name -> logical [X Y Z]), for the GUI.

veh = inp;

%% Calculated hardpoint coordinates (not inputs in the spreadsheet)
calcCoords = struct( ...
    'uprrwd_outbrdF', [false true false], 'lwrrwd_outbrdF', [false true false], ...
    'uprrwd_outbrdR', [false true false], 'lwrrwd_outbrdR', [false true false], ...
    'arb_upr_droplinkF', [false false true], 'arb_lwr_droplinkF', [false true false], 'arb_centreF', [false true true], ...
    'arb_upr_droplinkR', [false false true], 'arb_lwr_droplinkR', [false true false], 'arb_centreR', [false true true]);

% Rear wishbone chassis pickups sit at the same lateral position as the forward pickups
% (only used for drawing - the geometry calcs only use their X and Z)
veh.uprrwd_outbrdF(2) = veh.uprfwd_outbrdF(2);
veh.lwrrwd_outbrdF(2) = veh.lwrfwd_outbrdF(2);
veh.uprrwd_outbrdR(2) = veh.uprfwd_outbrdR(2);
veh.lwrrwd_outbrdR(2) = veh.lwrfwd_outbrdR(2);

% ARB links
veh.arb_upr_droplinkF(3) = veh.rocker_centreF(3);
veh.arb_lwr_droplinkF(2) = veh.arb_upr_droplinkF(2);
veh.arb_centreF(2) = 0;
veh.arb_centreF(3) = veh.arb_lwr_droplinkF(3);

veh.arb_upr_droplinkR(3) = veh.rocker_centreR(3);
veh.arb_lwr_droplinkR(2) = veh.arb_upr_droplinkR(2);
veh.arb_centreR(2) = 0;
veh.arb_centreR(3) = veh.arb_lwr_droplinkR(3);

veh.tyre_groundF = [0, veh.x_trackF / 2, 0];
veh.tyre_groundR = [-veh.x_wheelbase, veh.x_trackR / 2, 0];

%% Mass
veh.m_total = veh.m_vehicle + veh.m_driver; % kg
veh.r_mass_total = ((veh.r_vehicle_front_rear_mass * veh.m_vehicle) + (veh.r_driver_front_rear_mass * veh.m_driver)) / veh.m_total;
veh.h_total_cog = ((veh.h_vehicle_cog * veh.m_vehicle) + (veh.h_driver_cog * veh.m_driver)) / veh.m_total; % mm

veh.m_totalF = veh.m_total * veh.r_mass_total;       % kg
veh.m_totalR = veh.m_total * (1 - veh.r_mass_total); % kg

veh.m_unsprung = veh.m_unsprungF + veh.m_unsprungR;  % kg
veh.m_sprungF = veh.m_totalF - veh.m_unsprungF;      % kg
veh.m_sprungR = veh.m_totalR - veh.m_unsprungR;      % kg

%% Tyres
veh.k_TyreF = veh.k_TyreF_imperial * 4.4482216 / 25.4; % N/mm
veh.k_TyreR = veh.k_TyreR_imperial * 4.4482216 / 25.4; % N/mm
veh.x_Tyre_diameterF = 25.4 * veh.x_Tyre_diameterF_imperial; % mm
veh.x_Tyre_diameterR = 25.4 * veh.x_Tyre_diameterR_imperial; % mm

% Static loaded radius: tyre squashed by the static corner load (N) / tyre stiffness (N/mm)
veh.x_LoadedRadiusF = (veh.x_Tyre_diameterF / 2) - (veh.m_totalF / 2 * 9.81) / veh.k_TyreF; % mm
veh.x_LoadedRadiusR = (veh.x_Tyre_diameterR / 2) - (veh.m_totalR / 2 * 9.81) / veh.k_TyreR; % mm

%% Springs
veh.k_springF = veh.k_springF_imperial * 9.81 / 25.4 / 2.205; % N/mm
veh.k_springR = veh.k_springR_imperial * 9.81 / 25.4 / 2.205; % N/mm

if strcmp(veh.MR_settingF, "High"), veh.r_MRF = veh.r_MR_highF; else, veh.r_MRF = veh.r_MR_lowF; end
if strcmp(veh.MR_settingR, "High"), veh.r_MRR = veh.r_MR_highR; else, veh.r_MRR = veh.r_MR_lowR; end

veh.k_wheelF = veh.k_springF * veh.r_MRF^2; % N/mm
veh.k_wheelR = veh.k_springR * veh.r_MRR^2; % N/mm

%% Anti-roll bars ('Springs' sheet, Roll Springs)
% Bar length = 2 x droplink lateral position (blank front = 0)
veh.x_ARB_lengthF = 2 * zeroIfNaN(veh.arb_lwr_droplinkF(2)); % mm
veh.x_ARB_lengthR = 2 * zeroIfNaN(veh.arb_lwr_droplinkR(2)); % mm
veh.r_ARB_MRF = pickHighLow(veh.MR_settingF, veh.r_ARB_MR_highF, veh.r_ARB_MR_lowF);
veh.r_ARB_MRR = pickHighLow(veh.MR_settingR, veh.r_ARB_MR_highR, veh.r_ARB_MR_lowR);
veh.x_ARB_armF = pickSetting(veh.ARB_settingF, veh.x_ARB_arm_softF, veh.x_ARB_arm_mediumF, veh.x_ARB_arm_hardF);
veh.x_ARB_armR = pickSetting(veh.ARB_settingR, veh.x_ARB_arm_softR, veh.x_ARB_arm_mediumR, veh.x_ARB_arm_hardR);
% Linear roll resistance (N/mm). NB: the sheet divides by 16 for the front bar and 32 for the rear.
veh.k_ARB_linearF = torsionRate(materialG(veh, veh.ARB_materialF), veh.x_ARB_armF, veh.x_ARB_lengthF, veh.x_ARB_ODF, veh.x_ARB_IDF, 16);
veh.k_ARB_linearR = torsionRate(materialG(veh, veh.ARB_materialR), veh.x_ARB_armR, veh.x_ARB_lengthR, veh.x_ARB_ODR, veh.x_ARB_IDR, 32);
veh.k_ARB_rollF = veh.k_ARB_linearF * 1000 * (veh.x_trackF / 1000)^2 / 2 * veh.r_ARB_MRF^2 * pi / 180; % Nm/deg
veh.k_ARB_rollR = veh.k_ARB_linearR * 1000 * (veh.x_trackR / 1000)^2 / 2 * veh.r_ARB_MRR^2 * pi / 180; % Nm/deg

%% Heave springs ('Springs' sheet)
r_heave_softF   = pickHighLow(veh.MR_settingF, veh.r_heave_MR_soft_highF,   veh.r_heave_MR_soft_lowF);
r_heave_mediumF = pickHighLow(veh.MR_settingF, veh.r_heave_MR_medium_highF, veh.r_heave_MR_medium_lowF);
r_heave_hardF   = pickHighLow(veh.MR_settingF, veh.r_heave_MR_hard_highF,   veh.r_heave_MR_hard_lowF);
veh.r_heave_MRF = pickSetting(veh.heave_settingF, r_heave_softF, r_heave_mediumF, r_heave_hardF);
veh.r_heave_MRR = pickSetting(veh.heave_settingR, 0, 0, 0);   % rear heave motion ratios are blank in the sheet
veh.k_heave_linearF = torsionRate(materialG(veh, veh.heave_materialF), veh.x_heave_armF, veh.x_heave_lengthF, veh.x_heave_ODF, veh.x_heave_IDF, 32);
veh.k_heave_linearR = torsionRate(materialG(veh, veh.heave_materialR), veh.x_heave_armR, veh.x_heave_lengthR, veh.x_heave_ODR, veh.x_heave_IDR, 32);
veh.k_heaveF = veh.r_heave_MRF^2 * veh.k_heave_linearF; % N/mm
veh.k_heaveR = veh.r_heave_MRR^2 * veh.k_heave_linearR; % N/mm

%% Roll ('Roll' sheet)
veh.k_roll_springF = (veh.x_trackF / 1000 / 2)^2 * 2 * (pi / 180) * veh.k_wheelF * 1000; % Nm/deg
veh.k_roll_springR = (veh.x_trackR / 1000 / 2)^2 * 2 * (pi / 180) * veh.k_wheelR * 1000; % Nm/deg
veh.k_roll_sprungF = (veh.k_roll_springF + veh.k_ARB_rollF) * veh.r_roll_correctionF;    % Nm/deg
veh.k_roll_sprungR = (veh.k_roll_springR + veh.k_ARB_rollR) * veh.r_roll_correctionR;    % Nm/deg
veh.k_roll_tyreF = (veh.x_trackF / 1000)^2 / 2 * (pi / 180) * veh.k_TyreF * 1000;        % Nm/deg
veh.k_roll_tyreR = (veh.x_trackR / 1000)^2 / 2 * (pi / 180) * veh.k_TyreR * 1000;        % Nm/deg
veh.k_rollF = veh.k_roll_tyreF * veh.k_roll_sprungF / (veh.k_roll_tyreF + veh.k_roll_sprungF); % Nm/deg
veh.k_rollR = veh.k_roll_tyreR * veh.k_roll_sprungR / (veh.k_roll_tyreR + veh.k_roll_sprungR); % Nm/deg
veh.r_FLLD = veh.k_rollF / (veh.k_rollF + veh.k_rollR); % -, front lateral load distribution

%% Heave ('Heave & Pitch' sheet)
veh.k_heave_sprungF = veh.k_wheelF + veh.k_heaveF; % N/mm
veh.k_heave_sprungR = veh.k_wheelR + veh.k_heaveR; % N/mm
veh.k_heave_totalF = veh.k_heave_sprungF * veh.k_TyreF / (veh.k_heave_sprungF + veh.k_TyreF); % N/mm
veh.k_heave_totalR = veh.k_heave_sprungR * veh.k_TyreR / (veh.k_heave_sprungR + veh.k_TyreR); % N/mm

%% Suspension geometry outputs
veh.r_brake_splitR = 1 - veh.r_brake_splitF; % -

veh.geoF = suspension_geometry(veh.upr_uprightF, veh.lwr_uprightF, veh.uprfwd_outbrdF, veh.uprrwd_outbrdF, ...
    veh.lwrfwd_outbrdF, veh.lwrrwd_outbrdF, 0, veh.x_trackF / 2);
veh.geoR = suspension_geometry(veh.upr_uprightR, veh.lwr_uprightR, veh.uprfwd_outbrdR, veh.uprrwd_outbrdR, ...
    veh.lwrfwd_outbrdR, veh.lwrrwd_outbrdR, -veh.x_wheelbase, veh.x_trackR / 2);

% Front
veh.h_roll_centreF  = veh.geoF.h_roll_centre;    % mm
veh.x_caster_trailF = veh.geoF.x_caster_trail;   % mm
veh.x_scrub_radiusF = veh.geoF.x_scrub_radius;   % mm
veh.theta_KPIF      = veh.geoF.theta_KPI;        % deg
veh.theta_casterF   = veh.geoF.theta_caster;     % deg
veh.x_VSAL_F        = veh.geoF.x_VSAL;           % mm, virtual swing axle length
veh.ic_2DF          = veh.geoF.ic_front_view;    % [Y lateral, Z height] mm, front-view instant centre

% Rear
veh.h_roll_centreR  = veh.geoR.h_roll_centre;    % mm
veh.x_caster_trailR = veh.geoR.x_caster_trail;   % mm
veh.x_scrub_radiusR = veh.geoR.x_scrub_radius;   % mm
veh.theta_KPIR      = veh.geoR.theta_KPI;        % deg
veh.theta_casterR   = veh.geoR.theta_caster;     % deg
veh.x_VSAL_R        = veh.geoR.x_VSAL;           % mm, virtual swing axle length
veh.ic_2DR          = veh.geoR.ic_front_view;    % [Y lateral, Z height] mm, front-view instant centre

% Anti geometry from the side-view instant centre [x from axle, z]
% (formulas copied as-is from the spreadsheet, including tan(tan(...)); IFERROR(...,0) -> 0 if not finite)
icF = veh.geoF.ic_side_view;
icR = veh.geoR.ic_side_view;

veh.r_anti_diveF = -veh.x_wheelbase * veh.r_brake_splitF * tan(tan(icF(2) / icF(1))) / veh.h_total_cog;
if ~isfinite(veh.r_anti_diveF), veh.r_anti_diveF = 0; end

if strcmp(veh.brake_typeR, "Outboard")
    veh.r_anti_liftR = veh.x_wheelbase * veh.r_brake_splitR * tan(tan(icR(2) / icR(1))) / veh.h_total_cog;
else % "Inboard"
    veh.r_anti_liftR = veh.x_wheelbase * veh.r_brake_splitR * tan(tan((icR(2) - veh.x_LoadedRadiusR) / icR(1))) / veh.h_total_cog;
end
if ~isfinite(veh.r_anti_liftR), veh.r_anti_liftR = 0; end

veh.r_anti_squatR = veh.x_wheelbase * tan(tan((icR(2) - veh.x_LoadedRadiusR) / icR(1))) / veh.h_total_cog;
if ~isfinite(veh.r_anti_squatR), veh.r_anti_squatR = 0; end

%% Dampers
veh.x_shock_staticF = veh.x_shock_extended - (veh.m_sprungF / 2 * 9.81 / veh.k_wheelF); % mm, static length front
veh.x_shock_staticR = veh.x_shock_extended - (veh.m_sprungR / 2 * 9.81 / veh.k_wheelR); % mm, static length rear

%% SusProg upright points
% [X, Y, Z]: X = inboard from the wheel centre plane (incl. rim offset),
% Y = height above the wheel centre, Z = longitudinal from the axle
hF = veh.x_trackF / 2 + veh.x_rim_offset;
hR = veh.x_trackR / 2 + veh.x_rim_offset;
veh.susprog_upr_uprightF = [hF - veh.upr_uprightF(2), veh.upr_uprightF(3) - veh.x_LoadedRadiusF, veh.upr_uprightF(1)];
veh.susprog_toeF         = [hF - veh.toe_outbrdF(2),  veh.toe_outbrdF(3) - veh.x_LoadedRadiusF,  veh.toe_outbrdF(1)];
veh.susprog_lwr_uprightF = [hF - veh.lwr_uprightF(2), veh.lwr_uprightF(3) - veh.x_LoadedRadiusF, veh.lwr_uprightF(1)];
veh.susprog_upr_uprightR = [hR - veh.upr_uprightR(2), veh.upr_uprightR(3) - veh.x_LoadedRadiusR, veh.upr_uprightR(1) + veh.x_wheelbase];
veh.susprog_toeR         = [hR - veh.toe_outbrdR(2),  veh.toe_outbrdR(3) - veh.x_LoadedRadiusR,  veh.toe_outbrdR(1) + veh.x_wheelbase];
veh.susprog_lwr_uprightR = [hR - veh.lwr_uprightR(2), veh.lwr_uprightR(3) - veh.x_LoadedRadiusR, veh.lwr_uprightR(1) + veh.x_wheelbase];

%% Powertrain, acceleration sim and GGV ('HV Powertrain', 'Acceleration', 'Tyres & Load Transfer')
veh.pt = vehicle_powertrain(veh);
veh.ggv = vehicle_ggv(veh);
end


function geo = suspension_geometry(upr_upright, lwr_upright, uprfwd, uprrwd, lwrfwd, lwrrwd, x_axle, y_tyre)
% Suspension geometry outputs for one axle, following the 'Suspension Geometry' sheet.
% Points are [X longitudinal, Y lateral, Z height] in mm; x_axle = axle X position
% (0 front, -wheelbase rear); y_tyre = half track (tyre contact patch, lateral).

geo.x_axle = x_axle;
geo.y_tyre = y_tyre;

% Side view: wishbone slopes and their lines through the upright ball joints
s_upr = (uprfwd(3) - uprrwd(3)) / (uprfwd(1) - uprrwd(1));
s_lwr = (lwrrwd(3) - lwrfwd(3)) / (lwrrwd(1) - lwrfwd(1));
b_upr = upr_upright(3) - s_upr * (upr_upright(1) - x_axle);   % height of upper line at the axle
b_lwr = lwr_upright(3) - s_lwr * (lwr_upright(1) - x_axle);   % height of lower line at the axle
geo.side_view_lines = [s_upr, b_upr; s_lwr, b_lwr];           % z = s*(x - x_axle) + b

% Side-view instant centre [x from axle, z]
x_ic = (b_upr - b_lwr) / (s_lwr - s_upr);
geo.ic_side_view = [x_ic, s_lwr * x_ic + b_lwr];

% Front view: project the points onto the axle plane along the wishbone slopes ("2D Plane Conversion")
upr_upright_2D = [upr_upright(2), b_upr];
lwr_upright_2D = [lwr_upright(2), b_lwr];
upr_chassis_2D = [uprfwd(2), uprfwd(3) - s_upr * (uprfwd(1) - x_axle)];
lwr_chassis_2D = [lwrfwd(2), lwrfwd(3) - s_lwr * (lwrfwd(1) - x_axle)];
geo.front_view_points = [upr_upright_2D; lwr_upright_2D; upr_chassis_2D; lwr_chassis_2D];

m_upr = (upr_upright_2D(2) - upr_chassis_2D(2)) / (upr_upright_2D(1) - upr_chassis_2D(1));
c_upr = upr_chassis_2D(2) - m_upr * upr_chassis_2D(1);
m_lwr = (lwr_upright_2D(2) - lwr_chassis_2D(2)) / (lwr_upright_2D(1) - lwr_chassis_2D(1));
c_lwr = lwr_chassis_2D(2) - m_lwr * lwr_chassis_2D(1);

% Front-view instant centre [Y lateral, Z height]
y_ic = (c_lwr - c_upr) / (m_upr - m_lwr);
z_ic = m_upr * y_ic + c_upr;
geo.ic_front_view = [y_ic, z_ic];

geo.h_roll_centre = z_ic - (z_ic / (y_ic - y_tyre)) * y_ic;
geo.x_VSAL = y_tyre - y_ic;

% Steering (kingpin) axis through the two upright ball joints, evaluated at the ground
t_ground = -upr_upright(3) / (lwr_upright(3) - upr_upright(3));
geo.x_caster_trail = upr_upright(1) + t_ground * (lwr_upright(1) - upr_upright(1)) - x_axle;
geo.x_scrub_radius = y_tyre - (upr_upright(2) + t_ground * (lwr_upright(2) - upr_upright(2)));
geo.theta_KPI    = -atand((upr_upright(2) - lwr_upright(2)) / (upr_upright(3) - lwr_upright(3)));
geo.theta_caster = -atand((upr_upright(1) - lwr_upright(1)) / (upr_upright(3) - lwr_upright(3)));
end


function x = zeroIfNaN(x)
if isnan(x), x = 0; end
end

function v = pickHighLow(setting, high, low)
% Spreadsheet IFS(setting = "Low", low, setting = "High", high)
if strcmp(setting, "High"), v = high; else, v = low; end
end

function v = pickSetting(setting, soft, medium, hard)
% Spreadsheet IFS(setting = "Soft"/"Medium"/"Hard"/"Detached")
switch char(setting)
    case 'Soft',   v = soft;
    case 'Medium', v = medium;
    case 'Hard',   v = hard;
    otherwise,     v = 0;   % "Detached"
end
end

function k = torsionRate(G, arm, len, OD, ID, divisor)
% Torsion bar rate at the moment arm (N/mm), as in the 'Springs' sheet; IFERROR(...,0)
k = G * 1e9 / ((arm / 1000)^2 * ((len / 1000) / 2)) * pi * ((OD / 1000)^4 - (ID / 1000)^4) / divisor / 1000;
if ~isfinite(k), k = 0; end
end

function G = materialG(veh, name)
% Shear modulus (GPa) from the material table
names = cellfun(@char, {veh.materials.name}, 'UniformOutput', false);
idx = find(strcmp(names, char(name)), 1);
if isempty(idx), G = NaN; else, G = veh.materials(idx).G; end
end
