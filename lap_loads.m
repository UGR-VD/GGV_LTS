function L = lap_loads(veh, res)
% LAP_LOADS  Work back from a lap sim result to the loads and suspension state around the lap.
%   L = lap_loads(veh, res)
%
% VEH  vehicle model (vehicle_model_calc) the lap was run with
% RES  lap_sim result (uses s, vCar, gLatSim, gLongSim)
%
% At every track point the simulated speed and accelerations are applied to the car using
% the same load transfer model as vehicle_ggv:
%   - aero: ClF / ClR from the aeromap at the current ride heights (solved iteratively,
%     like the spreadsheet's "Input = Calc" circular solve), q = 0.5*rho*A*v^2
%   - longitudinal transfer M*ax*h/WB, split into a geometric part (anti-dive / anti-lift /
%     anti-squat) that skips the springs and an elastic part that goes through them
%   - lateral transfer per axle as the top-speed block of the GGV sheet: a geometric part
%     through the roll centres and an elastic part split by FLLD that rolls the car
% Ride heights use the per-wheel heave rate incl. tyre (k_heave_total), as vehicle_ggv does.
% Not modelled: aero drag pitch moment, unsprung mass, kinematic jacking, damper forces.
%
% Corner order everywhere is [FL FR RL RR]; +gLat = turning left (load goes to the right).
% Returns L with (1xN rows unless noted, N = track points):
%   s, v, ax, ay                     distance (m), speed (m/s), accelerations (m/s^2)
%   Fz            4xN  tyre vertical load (N)
%   FzStatic      4x1  static tyre load (N)
%   dFzLong, dFzLatF, dFzLatR        load transfers (N, per wheel)
%   aeroF, aeroR, ClF, ClR           aero downforce per axle (N) and Cl used
%   FRH, RRH                         dynamic ride heights (mm)
%   roll (deg, +ve = body leans right), pitch (deg, +ve = nose down)
%   wheelTravel   4xN  suspension travel at the wheel relative to static (mm, + = bump)
%   tyreDefl      4xN  tyre compression relative to static (mm, + = more squashed)
%   springTravel  4xN  coil spring / shock compression relative to static (mm, + = bump)
%   shockLength   4xN  shock length (mm), from veh.x_shock_static*
%   springForce   4xN  coil spring force (N)
%   shockBottomed, shockTopped, wheelLift   4xN logical flags
%   summary       table of min / max per quantity

g = 9.81;
N = numel(res.s);
v = res.vCar(:)';
ax = g * res.gLongSim(:)';
ay = g * res.gLatSim(:)';
v(~isfinite(v)) = NaN;

M = veh.m_total;
W = M * g;
wf = veh.r_mass_total;
h = veh.h_total_cog / 1000;       % m
WB = veh.x_wheelbase / 1000;      % m
tF = veh.x_trackF / 1000;         % m
tR = veh.x_trackR / 1000;         % m
rhoA = veh.rho_air * veh.x_aero_area;
RH0 = [veh.h_static_FRH, veh.h_static_RRH];

%% Static loads (per wheel)
FzStaticF = W * wf / 2;
FzStaticR = W * (1 - wf) / 2;

%% Longitudinal load transfer (per wheel), + = onto the rear
dLong = M * ax * h / WB / 2;
% Geometric share (anti features) does not go through the springs
antiF = zeros(1, N); antiR = zeros(1, N);
brk = ax < 0;
antiF(brk) = veh.r_anti_diveF;
antiR(brk) = veh.r_anti_liftR;
antiR(~brk) = veh.r_anti_squatR;
if strcmp(veh.drive_type, "AWD"), antiF(~brk) = 0; end   % no front anti-lift geometry in the model
dLongElF = dLong .* (1 - antiF);
dLongElR = dLong .* (1 - antiR);

%% Lateral load transfer (per wheel), + = onto the right-hand wheels
RCH = wf * veh.h_roll_centreF + (1 - wf) * veh.h_roll_centreR;   % mm, roll axis height at the CoG
hAbove = (veh.h_total_cog - RCH) / 1000;                          % m, roll moment arm
Fy = M * ay;
dLatGeoF = Fy * wf * veh.h_roll_centreF / 1000 / tF;
dLatGeoR = Fy * (1 - wf) * veh.h_roll_centreR / 1000 / tR;
dLatElF = Fy * hAbove * veh.r_FLLD / tF;
dLatElR = Fy * hAbove * (1 - veh.r_FLLD) / tR;
dLatF = dLatGeoF + dLatElF;
dLatR = dLatGeoR + dLatElR;

%% Roll: elastic roll moment / roll stiffness (Nm/deg)
rollMoment = Fy * hAbove;                                          % Nm
roll = rollMoment / (veh.k_rollF + veh.k_rollR);                   % deg, whole car incl. tyres
rollSuspF = dLatElF * tF / veh.k_roll_sprungF;                     % deg, suspension only (springs + ARB)
rollSuspR = dLatElR * tR / veh.k_roll_sprungR;

%% Aero + ride heights (fixed point on ClF, ClR at every point)
q = 0.5 * rhoA * v.^2;
ClF = repmat(aeroPoly(veh.aeromap_ClF, RH0(1), RH0(2)) * veh.r_aero_correctionF, 1, N);
ClR = repmat(aeroPoly(veh.aeromap_ClR, RH0(1), RH0(2)) * veh.r_aero_correctionR, 1, N);
for it = 1:200
    aeroF = q .* ClF;                    % N per axle
    aeroR = q .* ClR;
    [FRH, RRH] = rideHeights(aeroF, aeroR);
    ClFn = veh.r_aero_correctionF * aeroPoly(veh.aeromap_ClF, FRH, RRH);
    ClRn = veh.r_aero_correctionR * aeroPoly(veh.aeromap_ClR, FRH, RRH);
    d = max(abs([ClFn - ClF, ClRn - ClR]), [], 'omitnan');
    ClF = ClF + 0.7 * (ClFn - ClF);
    ClR = ClR + 0.7 * (ClRn - ClR);
    if d < 1e-10, break, end
end
aeroF = q .* ClF;
aeroR = q .* ClR;
[FRH, RRH, heaveF, heaveR] = rideHeights(aeroF, aeroR);

%% Corner loads [FL FR RL RR]
side = [-1; 1; -1; 1];                   % left / right
Fz = [FzStaticF - dLong + aeroF / 2 + side(1) * dLatF;
      FzStaticF - dLong + aeroF / 2 + side(2) * dLatF;
      FzStaticR + dLong + aeroR / 2 + side(3) * dLatR;
      FzStaticR + dLong + aeroR / 2 + side(4) * dLatR];

%% Suspension travel at the wheel (+ = bump), relative to static
% Heave / pitch through the springs (+ heave spring), roll through springs + ARB
heaveTravelF = (-dLongElF + aeroF / 2) / veh.k_heave_sprungF;
heaveTravelR = ( dLongElR + aeroR / 2) / veh.k_heave_sprungR;
rollTravelF = tF * 1000 / 2 * tand(rollSuspF);
rollTravelR = tR * 1000 / 2 * tand(rollSuspR);
wheelTravel = [heaveTravelF + side(1) * rollTravelF;
               heaveTravelF + side(2) * rollTravelF;
               heaveTravelR + side(3) * rollTravelR;
               heaveTravelR + side(4) * rollTravelR];

kT = [veh.k_TyreF; veh.k_TyreF; veh.k_TyreR; veh.k_TyreR];
FzStatic = [FzStaticF; FzStaticF; FzStaticR; FzStaticR];
tyreDefl = (Fz - FzStatic) ./ kT;

%% Coil spring / shock
MR = [veh.r_MRF; veh.r_MRF; veh.r_MRR; veh.r_MRR];
kS = [veh.k_springF; veh.k_springF; veh.k_springR; veh.k_springR];
springTravel = MR .* wheelTravel;                                   % mm
shockStatic = [veh.x_shock_staticF; veh.x_shock_staticF; veh.x_shock_staticR; veh.x_shock_staticR];
shockLength = shockStatic - springTravel;
% Static spring force from the sprung corner weight through the motion ratio
springStatic = [veh.m_sprungF; veh.m_sprungF; veh.m_sprungR; veh.m_sprungR] * g / 2 ./ MR;
springForce = springStatic + kS .* springTravel;

%% Pitch (+ = nose down) from the ride height change
pitch = atand(((RH0(1) - FRH) - (RH0(2) - RRH)) / veh.x_wheelbase);

%% Pack
L.s = res.s(:)';
L.v = v; L.ax = ax; L.ay = ay;
L.Fz = Fz;
L.FzStatic = FzStatic;
L.dFzLong = dLong;
L.dFzLatF = dLatF;
L.dFzLatR = dLatR;
L.aeroF = aeroF; L.aeroR = aeroR; L.ClF = ClF; L.ClR = ClR;
L.FRH = FRH; L.RRH = RRH;
L.heaveF = heaveF; L.heaveR = heaveR;
L.roll = roll;
L.rollSuspF = rollSuspF; L.rollSuspR = rollSuspR;
L.pitch = pitch;
L.wheelTravel = wheelTravel;
L.tyreDefl = tyreDefl;
L.springTravel = springTravel;
L.shockLength = shockLength;
L.springForce = springForce;
L.shockBottomed = shockLength < veh.x_shock_compressed;
L.shockTopped = shockLength > veh.x_shock_extended;
L.wheelLift = Fz <= 0;
L.corners = {'FL', 'FR', 'RL', 'RR'};
L.summary = makeSummary(L);

%% Nested: ride heights from the aero loads (as vehicle_ggv: per-wheel heave rate incl. tyre)
    function [frh, rrh, dF, dR] = rideHeights(aF, aR)
        % Axle-average load change per wheel (lateral transfer cancels across the axle)
        dF = -dLong + aF / 2;
        dR =  dLong + aR / 2;
        frh = RH0(1) - dF / veh.k_heave_totalF;
        rrh = RH0(2) - dR / veh.k_heave_totalR;
    end
end


function T = makeSummary(L)
names = {}; mins = []; maxs = []; units = {};
add('Speed', L.v, 'm/s');
add('Front ride height', L.FRH, 'mm');
add('Rear ride height', L.RRH, 'mm');
add('Roll angle', L.roll, 'deg');
add('Pitch angle', L.pitch, 'deg');
add('Front downforce', L.aeroF, 'N');
add('Rear downforce', L.aeroR, 'N');
for k = 1:4
    add(['Tyre load ' L.corners{k}], L.Fz(k, :), 'N');
end
for k = 1:4
    add(['Wheel travel ' L.corners{k}], L.wheelTravel(k, :), 'mm');
end
for k = 1:4
    add(['Shock length ' L.corners{k}], L.shockLength(k, :), 'mm');
end
for k = 1:4
    add(['Spring force ' L.corners{k}], L.springForce(k, :), 'N');
end
T = table(mins(:), maxs(:), units(:), 'VariableNames', {'Min', 'Max', 'Unit'}, 'RowNames', names(:));

    function add(n, x, u)
        names{end + 1} = n; mins(end + 1) = min(x, [], 'omitnan'); maxs(end + 1) = max(x, [], 'omitnan'); units{end + 1} = u;
    end
end

function Cl = aeroPoly(c, f, r)
% Same polynomial as vehicle_ggv: c0 + c1*f + c2*r + c3*f*r + c4*f^2 + c5*r^2 + c6*f^3 + c7*r^3 + c8*f^2*r + c9*f*r^2
Cl = c(1) + c(2) * f + c(3) * r + c(4) * f .* r + c(5) * f.^2 + c(6) * r.^2 + c(7) * f.^3 + c(8) * r.^3 ...
    + c(9) * f.^2 .* r + c(10) * f .* r.^2;
end
