function res = lap_sim(ggv, trackFile, opts)
% LAP_SIM  Quasi-steady lap time solver (the Solver_Script algorithm as a function).
%   res = lap_sim(ggv, trackFile)
%   res = lap_sim(ggv, trackFile, opts)
%
% GGV       struct with fields v, gLat, gLong_acc, gLong_brk, gLong_powtrn (e.g. veh.ggv)
% TRACKFILE .mat file with Car_Coord_X / Car_Coord_Y (Time/Value structs), e.g. a logged lap.
%           If it also has Ground_Speed, CG_Accel_Lateral and CG_Accel_Longitudinal, the logged
%           lap is returned in res.logged for comparison.
% OPTS      (optional) trackSpacing (0.5 m), closeBlend (30 m), curvSmoothPts (1),
%           minPeakProminence (40 m), minPeakHeight (-400)
%
% RES fields: x, y, s, L, radius, apex_s, apex_radius, vMaxAtApex, vLimit, vCar, ayCar,
%             gLatSim (signed), gLongSim, tSim, lapTime, solveTime, nGaps, logged (or [])

if nargin < 3, opts = struct(); end
opts = setDefault(opts, 'trackSpacing', 0.5);
opts = setDefault(opts, 'closeBlend', 30);
opts = setDefault(opts, 'curvSmoothPts', 1);
opts = setDefault(opts, 'minPeakProminence', 40);
opts = setDefault(opts, 'minPeakHeight', -400);
g0 = 9.81;

%% GGV surface (as Solver_Script)
v = ggv.v;
speedRings = linspace(min(v), max(v), 12);
phi = linspace(-pi, pi, 400);
[Phi, V] = meshgrid(phi, speedRings);
ayMax = g0 * interp1(v, ggv.gLat, V, 'pchip');
axGrip = g0 * interp1(v, ggv.gLong_acc, V, 'pchip');
axPowertrain = g0 * interp1(v, ggv.gLong_powtrn, V, 'pchip');
axBrk = g0 * interp1(v, ggv.gLong_brk, V, 'pchip');
axFwd = min(axGrip, axPowertrain);
Ax = zeros(size(V));
Ay = ayMax .* sin(Phi);
forward = cos(Phi) >= 0;
Ax(forward) = axFwd(forward) .* cos(Phi(forward));
Ax(~forward) = -axBrk(~forward) .* abs(cos(Phi(~forward)));
res.speedRings = speedRings;

%% Track: resample at a fixed spacing and close the loop smoothly
trackData = load(trackFile);
xRaw = trackData.Car_Coord_X.Value(:)';
yRaw = trackData.Car_Coord_Y.Value(:)';
sRaw = [0, cumsum(hypot(diff(xRaw), diff(yRaw)))];
[sRaw, rawIdx] = unique(sRaw);
sTrackRaw = 0:opts.trackSpacing:sRaw(end);
xTrack = interp1(sRaw, xRaw(rawIdx), sTrackRaw);
yTrack = interp1(sRaw, yRaw(rawIdx), sTrackRaw);

Lc = sTrackRaw(end) + opts.trackSpacing;
j  = find(sTrackRaw >= 3, 1);
h  = [xTrack(j) - xTrack(1), yTrack(j) - yTrack(1)];
h  = h / norm(h);
w  = min(max((sTrackRaw - (sTrackRaw(end) - opts.closeBlend)) / opts.closeBlend, 0), 1);
w  = 0.5 * (1 - cos(pi * w));
xTrack = (1 - w) .* xTrack + w .* (xTrack(1) - (Lc - sTrackRaw) * h(1));
yTrack = (1 - w) .* yTrack + w .* (yTrack(1) - (Lc - sTrackRaw) * h(2));

%% Curvature
sLap = [0, cumsum(hypot(diff(xTrack), diff(yTrack)))];
L = sLap(end) + hypot(xTrack(1) - xTrack(end), yTrack(1) - yTrack(end));
sPrev = circshift(sLap, 1);  sPrev(1) = sLap(end) - L;
sNext = circshift(sLap, -1); sNext(end) = sLap(1) + L;
dx = (circshift(xTrack, -1) - circshift(xTrack, 1)) ./ (sNext - sPrev);
dy = (circshift(yTrack, -1) - circshift(yTrack, 1)) ./ (sNext - sPrev);
ddx = (circshift(dx, -1) - circshift(dx, 1)) ./ (sNext - sPrev);
ddy = (circshift(dy, -1) - circshift(dy, 1)) ./ (sNext - sPrev);

kappa = abs(dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^(3/2);
if opts.curvSmoothPts > 1, kappa = movmean(kappa, opts.curvSmoothPts); end
kappa = max(kappa, eps);
radius = min(1 ./ kappa, 500);
radius(~isfinite(radius)) = NaN;

%% Apexes
[apex_radius, apex_sLaps] = findApexes(radius, sLap, opts.minPeakProminence, opts.minPeakHeight);

%% Apex speeds and cornering speed limit
vGrid = linspace(0, max(speedRings), 100);
ayEnvelope = max(Ay, [], 2);
ayAvailGrid = interp1(speedRings(:), ayEnvelope(:), vGrid, 'pchip', 'extrap');
vMaxAtApex = NaN(size(apex_radius));
for k = 1:numel(apex_sLaps)
    idx = find(vGrid.^2 ./ apex_radius(k) <= ayAvailGrid, 1, 'last');
    if ~isempty(idx), vMaxAtApex(k) = vGrid(idx); end
end

vFine = linspace(0, max(speedRings), 1000);
ayAvailFine = interp1(speedRings(:), ayEnvelope(:), vFine, 'pchip');
feasibleV = (vFine.^2) ./ radius(:) <= ayAvailFine;
[~, lastIdx] = max(feasibleV .* (1:numel(vFine)), [], 2);
vLimit = vFine(lastIdx);

%% GGV slices on a fine speed grid
ggvDv = 0.01;
vTab  = (0:ggvDv:max(speedRings) + ggvDv)';
AxTab = interp1(speedRings(:), Ax, vTab, 'pchip', 'extrap');
AyTab = interp1(speedRings(:), Ay, vTab, 'pchip', 'extrap');

%% Solve for vCar
solveTimer = tic;
N = numel(sLap);
vCar = inf(1, N);

for i = 1:numel(apex_sLaps)
    [~, apex_idx] = min(abs(sLap - apex_sLaps(i)));
    candidate = inf(1, N);
    candidate(apex_idx) = vMaxAtApex(i);

    for n = apex_idx + 1:N
        vel = candidate(n - 1);
        if ~isfinite(vel), break, end
        candidate(n) = stepSpeed(vel, radius(n), true, sLap(n) - sLap(n - 1), vLimit(n));
        % Early stop: an earlier apex's acceleration pass is already slower here
        if candidate(n) > vCar(n), break, end
    end

    for n = apex_idx - 1:-1:1
        vel = candidate(n + 1);
        if ~isfinite(vel), break, end
        candidate(n) = stepSpeed(vel, radius(n), false, sLap(n + 1) - sLap(n), vLimit(n));
    end

    better = isfinite(candidate) & candidate < vCar;
    vCar(better) = candidate(better);
end

% Wrap-around: accelerate from sLap(end) through sLap(1)
if ~isempty(apex_sLaps) && isfinite(vCar(end))
    [~, firstApexIdx] = min(abs(sLap - min(apex_sLaps)));
    startCandidate = inf(1, N);
    prevVel = vCar(end);
    for n = 1:N
        if n == 1, ds = L - sLap(end); else, ds = sLap(n) - sLap(n - 1); end
        startCandidate(n) = stepSpeed(prevVel, radius(n), true, ds, vLimit(n));
        prevVel = startCandidate(n);
        % Carry on past the first apex until an existing (slower) pass takes over, so an
        % apex that does not limit the speed cannot leave a speed spike near the line
        if n >= firstApexIdx && startCandidate(n) > vCar(n), break, end
    end
    better = isfinite(startCandidate) & startCandidate < vCar;
    vCar(better) = startCandidate(better);
end
vCar(isinf(vCar)) = NaN;
res.solveTime = toc(solveTimer);

%% Results
ayCar = vCar.^2 ./ radius;
turnDir = sign(movmean(dx .* ddy - dy .* ddx, max(3, round(5 / opts.trackSpacing))));
segLen = [diff(sLap), L - sLap(end)];
dtSim = segLen ./ ((vCar + circshift(vCar, -1)) / 2);

res.x = xTrack;
res.y = yTrack;
res.s = sLap;
res.L = L;
res.radius = radius;
res.apex_s = apex_sLaps;
res.apex_radius = apex_radius;
res.vMaxAtApex = vMaxAtApex;
res.vLimit = vLimit;
res.vCar = vCar;
res.ayCar = ayCar;
res.gLatSim = turnDir .* ayCar / g0;              % signed like the logger (+ve = turning left)
res.gLongSim = gradient(vCar.^2 / 2, sLap) / g0;  % a_x = d(v^2/2)/ds
res.tSim = [0, cumsum(dtSim(1:end - 1))];
res.lapTime = sum(dtSim);
res.nGaps = nnz(isnan(vCar));

%% Logged lap (if the file has it), on the same points
res.logged = [];
if all(isfield(trackData, {'Ground_Speed', 'CG_Accel_Lateral', 'CG_Accel_Longitudinal'}))
    toTrack = @(sig) interp1(sRaw, sig(rawIdx), sTrackRaw);
    lg.v     = toTrack(trackData.Ground_Speed.Value(:)') / 3.6;                       % km/h -> m/s
    lg.gLat  = toTrack(movmean(trackData.CG_Accel_Lateral.Value(:)', 5));             % g
    lg.gLong = toTrack(movmean(trackData.CG_Accel_Longitudinal.Value(:)', 5));        % g
    t = toTrack(trackData.Ground_Speed.Time(:)');
    lg.t = t - t(1);
    lg.lapTime = trackData.Ground_Speed.Time(end) - trackData.Ground_Speed.Time(1);
    res.logged = lg;
end

%% Nested step function (uses the GGV tables)
    function vNew = stepSpeed(vel, R, isAccel, ds, vLim)
        f = vel / ggvDv;
        k = min(floor(f) + 1, size(AxTab, 1) - 1);
        wt = f - (k - 1);
        axAtVel = AxTab(k, :) * (1 - wt) + AxTab(k + 1, :) * wt;
        ayAtVel = AyTab(k, :) * (1 - wt) + AyTab(k + 1, :) * wt;
        feasible = ayAtVel >= vel.^2 ./ R;
        if any(feasible)
            if isAccel
                ax = max(axAtVel(feasible));
                vNew = sqrt(max(0, vel.^2 + 2 * ax * ds));
            else
                ax = min(axAtVel(feasible));
                vNew = sqrt(max(0, vel.^2 - 2 * ax * ds));
            end
            if vNew > vLim, vNew = vLim; end
        else
            vNew = vLim;   % out of grip: car sits at its cornering limit
        end
    end
end


function opts = setDefault(opts, name, value)
if ~isfield(opts, name), opts.(name) = value; end
end

function [pk, loc] = findApexes(radius, s, prominence, height)
% Corner apexes = minima of the radius: findpeaks(-radius, s, 'MinPeakProminence', .., 'MinPeakHeight', ..)
if exist('findpeaks', 'file') == 2 && ~exist('OCTAVE_VERSION', 'builtin')
    [pk, loc] = findpeaks(-radius, s, 'MinPeakProminence', prominence, 'MinPeakHeight', height);
    pk = -pk;
    return
end
% Fallback (no Signal Processing Toolbox): same peak / prominence definition as findpeaks
y = -radius(:)';
n = numel(y);
idx = [];
i = 2;
while i < n
    if y(i) > y(i - 1)
        j = i;
        while j < n && y(j + 1) == y(i), j = j + 1; end   % plateau: take its left edge
        if j < n && y(j + 1) < y(i), idx(end + 1) = i; end %#ok<AGROW>
        i = j + 1;
    else
        i = i + 1;
    end
end
keep = false(size(idx));
for k = 1:numel(idx)
    p = idx(k);
    if y(p) < height, continue, end
    l = find(y(1:p - 1) > y(p), 1, 'last'); if isempty(l), l = 0; end
    r = find(y(p + 1:end) > y(p), 1) + p;   if isempty(r), r = n + 1; end
    leftMin = min(y(l + 1:p));
    rightMin = min(y(p:r - 1));
    keep(k) = y(p) - max(leftMin, rightMin) >= prominence;
end
idx = idx(keep);
pk = radius(idx);
loc = s(idx);
end
