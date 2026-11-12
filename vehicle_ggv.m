function ggv = vehicle_ggv(veh)
% VEHICLE_GGV  GGV data from the 'Tyres & Load Transfer' sheets of UGR27_Vehicle_Dynamics_Spreadsheet.
% For each speed it solves the cornering, braking and acceleration load-transfer blocks.
% The spreadsheet "circular solves" these by matching each 'Input' cell (tyre friction
% coefficients and aero Cl) to its 'Calc' cell; here that is a fixed-point iteration.
%
% Speeds (as in the sheet): 0, top/4, top/2, skid pad, 3*top/4, top speed.
% The top-speed block uses the sheet's fixed-g geometric load transfer method (see topSpeed below).
%
% Returns GGV with fields (one value per speed):
%   v, gLat, gLong_acc, gLong_brk, gLong_powtrn  - what Solver_Script needs (g)
%   gLat_theoretical, balance, brake_bias, and per-mode aero/ride height details

g = 9.81;
c.M = veh.m_total;
c.W = veh.m_total * g;
c.wf = veh.r_mass_total;
c.h = veh.h_total_cog;
c.WB = veh.x_wheelbase;
c.tF = veh.x_trackF;
c.tR = veh.x_trackR;
c.FLLD = veh.r_FLLD;
c.kF = veh.k_heave_totalF;
c.kR = veh.k_heave_totalR;
c.RH0 = [veh.h_static_FRH, veh.h_static_RRH];
c.rhoA = veh.rho_air * veh.x_aero_area;
c.RWD = strcmp(veh.drive_type, "RWD");
c.mu_lat  = @(Fz) (veh.r_mu_lat_gradient  * Fz + veh.r_mu_lat_0)  * veh.r_mu_lat_scale;
c.mu_long = @(Fz) (veh.r_mu_long_gradient * Fz + veh.r_mu_long_0) * veh.r_mu_long_scale;
c.ClF = @(f, r) veh.r_aero_correctionF * aeroPoly(veh.aeromap_ClF, f, r);
c.ClR = @(f, r) veh.r_aero_correctionR * aeroPoly(veh.aeromap_ClR, f, r);
c.hRCF = veh.h_roll_centreF;
c.hRCR = veh.h_roll_centreR;
c.gLatTop = veh.g_lat_top_speed_load_transfer;

v_top = veh.pt.v_top;

% Skid pad speed: v = sqrt(aLat(v) * g * R)  (the sheet's speed cell is typed in; here it is solved)
v_skid = sqrt(1.7 * g * veh.x_skidpad_radius);
for it = 1:100 * isnan(veh.v_skidpad)
    cb = cornerBlock(c, v_skid, false);
    v_new = sqrt(cb.gLat * g * veh.x_skidpad_radius);
    if abs(v_new - v_skid) < 1e-12, v_skid = v_new; break, end
    v_skid = v_new;
end

if ~isnan(veh.v_skidpad), v_skid = veh.v_skidpad; end
speeds = [0, v_top / 4, v_top / 2, v_skid, 3 * v_top / 4, v_top];
labels = {'Zero', 'Low', 'Medium', 'Skid pad', 'High', 'Top'};
n = numel(speeds);

ggv.v = speeds;
ggv.labels = labels;
ggv.v_skidpad = v_skid;
fields = {'gLat', 'gLat_theoretical', 'balance', 'gLong_brk', 'brake_bias', 'gLong_acc', 'gLong_powtrn', ...
    'corner_ClF', 'corner_ClR', 'corner_FRH', 'corner_RRH', 'brake_ClF', 'brake_ClR', 'brake_FRH', 'brake_RRH', ...
    'accel_ClF', 'accel_ClR', 'accel_FRH', 'accel_RRH'};
for k = 1:numel(fields), ggv.(fields{k}) = NaN(1, n); end

for k = 1:n
    isTop = (k == n);
    cb = cornerBlock(c, speeds(k), isTop);
    bb = brakeBlock(c, speeds(k));
    ab = accelBlock(c, speeds(k));
    ggv.gLat(k) = cb.gLat;
    ggv.gLat_theoretical(k) = cb.gLat_theoretical;
    ggv.balance(k) = cb.balance;
    ggv.corner_ClF(k) = cb.ClF; ggv.corner_ClR(k) = cb.ClR; ggv.corner_FRH(k) = cb.FRH; ggv.corner_RRH(k) = cb.RRH;
    ggv.gLong_brk(k) = bb.gLong;
    ggv.brake_bias(k) = bb.bias;
    ggv.brake_ClF(k) = bb.ClF; ggv.brake_ClR(k) = bb.ClR; ggv.brake_FRH(k) = bb.FRH; ggv.brake_RRH(k) = bb.RRH;
    ggv.gLong_acc(k) = ab.gLong;
    ggv.accel_ClF(k) = ab.ClF; ggv.accel_ClR(k) = ab.ClR; ggv.accel_FRH(k) = ab.FRH; ggv.accel_RRH(k) = ab.RRH;
    if isTop
        ggv.gLong_powtrn(k) = 0;   % typed in as 0 in the sheet (no drive force left at top speed)
    else
        ggv.gLong_powtrn(k) = powertrainAccel(veh.pt, speeds(k)) / g;
    end
end
ggv.corner_aero_balance = ggv.corner_ClF ./ (ggv.corner_ClF + ggv.corner_ClR);
ggv.brake_aero_balance = ggv.brake_ClF ./ (ggv.brake_ClF + ggv.brake_ClR);
ggv.accel_aero_balance = ggv.accel_ClF ./ (ggv.accel_ClF + ggv.accel_ClR);
end


%% ---------------- Blocks ----------------

function out = cornerBlock(c, v, topSpeed)
% Front/Rear Cornering Load Transfer blocks. Unknowns: [muOF muIF muOR muIR ClF ClR]
q = 0.5 * c.rhoA * v^2;
x0 = [c.mu_lat(c.W * c.wf / 2), c.mu_lat(c.W * c.wf / 2), c.mu_lat(c.W * (1 - c.wf) / 2), ...
      c.mu_lat(c.W * (1 - c.wf) / 2), c.ClF(c.RH0(1), c.RH0(2)), c.ClR(c.RH0(1), c.RH0(2))];
[x, s] = fixedPoint(@(x) cornerEval(c, q, x, topSpeed), x0);

gripF = s.FzFo * x(1) + s.FzFi * x(2);
gripR = s.FzRo * x(3) + s.FzRi * x(4);
yawF = gripF * (1 - c.wf) / 1000 * c.WB;
yawR = gripR * c.wf / 1000 * c.WB;
used = min(yawF, yawR);
usedF = used / ((1 - c.wf) / 1000 * c.WB);
usedR = used / (c.wf / 1000 * c.WB);
out.gLat = (usedF + usedR) / c.M / 9.81;
out.gLat_theoretical = (gripF + gripR) / c.M / 9.81;
out.balance = (1 - 2 * used / (yawF + yawR)) * sign(yawR - yawF);
out.ClF = x(5); out.ClR = x(6); out.FRH = s.FRH; out.RRH = s.RRH;
out.Fz = [s.FzFo, s.FzFi, s.FzRo, s.FzRi];
end

function [xn, s] = cornerEval(c, q, x, topSpeed)
muOF = x(1); muIF = x(2); muOR = x(3); muIR = x(4); ClF = x(5); ClR = x(6);
h = c.h / 1000;
WF = c.W * c.wf;
WR = c.W * (1 - c.wf);
aeroF = q * ClF / 2;   % per tyre
aeroR = q * ClR / 2;
if ~topSpeed
    % Friction-based load split per axle, then redistributed by FLLD; aero added per tyre
    LFo = (c.tF / 2000 * WF + muIF * h * WF) / (c.tF / 1000 - muOF * h + muIF * h);
    LFi = WF - LFo;
    LRo = (c.tR / 2000 * WR + muIR * h * WR) / (c.tR / 1000 - muOR * h + muIR * h);
    LRi = WR - LRo;
    dTot = (LFo - LFi) + (LRo - LRi);
    totF = dTot * c.FLLD;
    totR = dTot * (1 - c.FLLD);
    s.FzFo = (LFo + LFi) / 2 + totF / 2 + aeroF;
    s.FzFi = (LFo + LFi) / 2 - totF / 2 + aeroF;
    s.FzRo = (LRo + LRi) / 2 + totR / 2 + aeroR;
    s.FzRi = (LRo + LRi) / 2 - totR / 2 + aeroR;
else
    % Top-speed block ("for calculations throughout Excel"): aero inside the axle load,
    % lateral transfer from a fixed lateral g (geometric + elastic via roll centres)
    LFo = (c.tF / 2000 * (WF + 2 * aeroF) + muIF * h * (WF + 2 * aeroF)) / (c.tF / 1000 - muOF * h + muIF * h);
    LFi = WF + 2 * aeroF - LFo;
    LRo = (c.tR / 2000 * (WR + 2 * aeroR) + muIR * h * (WR + 2 * aeroR)) / (c.tR / 1000 - muOR * h + muIR * h);
    LRi = WR + 2 * aeroR - LRo;
    RCH = c.wf * c.hRCF + (1 - c.wf) * c.hRCR;
    hAbove = c.h - RCH;
    Fy = c.gLatTop * 9.81 * c.M;
    totF = 2 * Fy * (c.wf * c.hRCF + hAbove * c.FLLD) / c.tF;
    totR = 2 * Fy * ((1 - c.wf) * c.hRCR + hAbove * (1 - c.FLLD)) / c.tR;
    s.FzFo = (LFo + LFi) / 2 + totF / 2;
    s.FzFi = (LFo + LFi) / 2 - totF / 2;
    s.FzRo = (LRo + LRi) / 2 + totR / 2;
    s.FzRi = (LRo + LRi) / 2 - totR / 2;
end
s.FRH = c.RH0(1) + WF * 0.5 / c.kF - (s.FzFo + s.FzFi) / 2 / c.kF;
s.RRH = c.RH0(2) + WR * 0.5 / c.kR - (s.FzRo + s.FzRi) / 2 / c.kR;
xn = [c.mu_lat(s.FzFo), c.mu_lat(s.FzFi), c.mu_lat(s.FzRo), c.mu_lat(s.FzRi), ...
      c.ClF(s.FRH, s.RRH), c.ClR(s.FRH, s.RRH)];
end

function out = brakeBlock(c, v)
% Braking + Aero block. Unknowns: [muF muR ClF ClR]
q = 0.5 * c.rhoA * v^2;
x0 = [c.mu_long(c.W * c.wf / 2), c.mu_long(c.W * (1 - c.wf) / 2), c.ClF(c.RH0(1), c.RH0(2)), c.ClR(c.RH0(1), c.RH0(2))];
[x, s] = fixedPoint(@(x) longEval(c, q, x, 'brake'), x0);
gripF = x(1) * s.FzF * 2;
gripR = x(2) * s.FzR * 2;
out.gLong = (gripF + gripR) / c.M / 9.81;
out.bias = gripF / (gripF + gripR);
out.ClF = x(3); out.ClR = x(4); out.FRH = s.FRH; out.RRH = s.RRH;
end

function out = accelBlock(c, v)
% Acceleration + Aero block. Unknowns: [muF muR ClF ClR]
q = 0.5 * c.rhoA * v^2;
x0 = [c.mu_long(c.W * c.wf / 2), c.mu_long(c.W * (1 - c.wf) / 2), c.ClF(c.RH0(1), c.RH0(2)), c.ClR(c.RH0(1), c.RH0(2))];
[x, s] = fixedPoint(@(x) longEval(c, q, x, 'accel'), x0);
if c.RWD, gripF = 0; else, gripF = s.FzF * x(1) * 2; end
gripR = x(2) * s.FzR * 2;
out.gLong = (gripF + gripR) / c.M / 9.81;
out.ClF = x(3); out.ClR = x(4); out.FRH = s.FRH; out.RRH = s.RRH;
end

function [xn, s] = longEval(c, q, x, mode)
muF = x(1); muR = x(2); ClF = x(3); ClR = x(4);
h = c.h / 1000;
WB = c.WB / 1000;
if strcmp(mode, 'brake')
    LF = (c.W / 2 * (muR * h + WB * c.wf)) / (WB + muR * h - muF * h);          % single front tyre
elseif c.RWD
    LF = (c.W / 2 * (WB * c.wf - muR * h)) / (WB - muR * h);
else
    LF = (c.W / 2 * (WB * c.wf - muR * h)) / (WB - muR * h + muF * h);
end
LR = c.W / 2 - LF;
s.FzF = LF + q * ClF / 2;
s.FzR = LR + q * ClR / 2;
s.FRH = c.RH0(1) + c.W * c.wf * 0.5 / c.kF - s.FzF / c.kF;
s.RRH = c.RH0(2) + c.W * (1 - c.wf) * 0.5 / c.kR - s.FzR / c.kR;
xn = [c.mu_long(s.FzF), c.mu_long(s.FzR), c.ClF(s.FRH, s.RRH), c.ClR(s.FRH, s.RRH)];
end


%% ---------------- Helpers ----------------

function [x, s] = fixedPoint(fun, x0)
% Solve x = fun(x) (the spreadsheet's "Input = Calc" matching). Falls back to damping if needed.
for relax = [1, 0.5, 0.2]
    x = x0;
    for it = 1:2000
        [xn, s] = fun(x);
        if any(~isfinite(xn)), break, end
        if max(abs(xn - x)) < 1e-13
            x = xn; [~, s] = fun(x);
            return
        end
        x = x + relax * (xn - x);
    end
end
warning('vehicle_ggv:noConvergence', 'Load transfer iteration did not converge.');
[~, s] = fun(x);
end

function Cl = aeroPoly(c, f, r)
% c0 + c1*f + c2*r + c3*f*r + c4*f^2 + c5*r^2 + c6*f^3 + c7*r^3 + c8*f^2*r + c9*f*r^2
Cl = c(1) + c(2) * f + c(3) * r + c(4) * f * r + c(5) * f^2 + c(6) * r^2 + c(7) * f^3 + c(8) * r^3 ...
    + c(9) * f^2 * r + c(10) * f * r^2;
end

function a = powertrainAccel(pt, v)
% XLOOKUP(v, Acceleration!AI, Acceleration!AH, , 1): first sim step with speed >= v
cand = find(pt.v >= v);
if isempty(cand)
    a = 0;   % never reached (the sheet has 0 typed in at top speed)
else
    [~, j] = min(pt.v(cand));   % smallest speed >= v (first if tied)
    a = pt.a(cand(j));
end
end
