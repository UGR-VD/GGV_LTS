%% GGV Surface Generation
speedRings = linspace(0, 60, 12);   % number of rings
phi = linspace(-pi, pi, 400);

[Phi, V] = meshgrid(phi, speedRings);

ayMax = 9 + 0.06*V;
axFwd = 6.5 + 0.03*V;
mask = V > 15;   
axFwd(mask) = max(0, 6.95 - 0.08*(V(mask) - 15));

axBrk = 3 + 0.02*V;

Ax = zeros(size(V));
Ay = ayMax .* sin(Phi);

forward = cos(Phi) >= 0;
Ax(forward) = axFwd(forward) .* cos(Phi(forward));
Ax(~forward) = -axBrk(~forward) .* abs(cos(Phi(~forward)));

figure
surf(Ax, Ay, V, V, 'EdgeColor', [0.6 0.6 0.6], 'FaceColor', 'interp')
shading interp
grid on
xlabel('Longitudinal accel (m/s^2)')
ylabel('Lateral accel (m/s^2)')
zlabel('Speed (m/s)')
title('g-g-v Surface from Discrete Rings')
view(3)
colorbar

hold on
for k = 1:numel(speedRings)
    plot3(Ax(k,:), Ay(k,:), V(k,:), 'k-')
end
hold off

%% GGV Surface from Vehicle Data
clear; clc;

g0 = 9.81; % m/s^2 per g

%% Input data
useVehicleModel = true;   % true = GGV calculated by the vehicle model, false = values typed in below

if useVehicleModel
    % Needs vehicle_model_defaults.m, vehicle_model_calc.m, vehicle_ggv.m, vehicle_powertrain.m on the path
    inp = vehicle_model_defaults();
    veh = vehicle_model_calc(inp);
    v            = veh.ggv.v;              % m/s: 0, top/4, top/2, skid pad, 3*top/4, top speed
    gLat         = veh.ggv.gLat;
    gLong_acc    = veh.ggv.gLong_acc;      % Grip-limited acceleration
    gLong_brk    = veh.ggv.gLong_brk;      % Grip-limited braking
    gLong_powtrn = veh.ggv.gLong_powtrn;   % Powertrain-limited acceleration
else
    v = [0, 5, 10, 12, 15, 20.1]; % m/s
    gLat = [1.57, 1.60, 1.69, 1.75, 1.84, 1.99];
    gLong_acc = [1.31, 1.33, 1.39, 1.43, 1.49, 1.62];    % Grip-limited acceleration
    gLong_brk = [1.75, 1.78, 1.89, 1.96, 2.07, 2.32];    % Grip-limited braking
    gLong_powtrn = [0.57, 0.57, 0.55, 0.53, 0.51, 0]; % Powertrain-limited acceleration
end

%% Build the speed/angle grid
speedRings = linspace(min(v), max(v), 12); % stays within the data range
phi = linspace(-pi, pi, 400);
[Phi, V] = meshgrid(phi, speedRings);

%% Interpolate limits onto the grid and convert g to m/s^2
ayMax = g0 * interp1(v, gLat, V, 'pchip');
axGrip = g0 * interp1(v, gLong_acc, V, 'pchip');
axPowertrain = g0 * interp1(v, gLong_powtrn, V, 'pchip');
axBrk = g0 * interp1(v, gLong_brk, V, 'pchip');

% Forward acceleration is limited by whichever is lower at each speed
axFwd = min(axGrip, axPowertrain);

%% Calculate the g-g-v surface
Ax = zeros(size(V));
Ay = ayMax .* sin(Phi);
forward = cos(Phi) >= 0;
Ax(forward) = axFwd(forward) .* cos(Phi(forward));
Ax(~forward) = -axBrk(~forward) .* abs(cos(Phi(~forward)));

%% Plot

Ax_plot = Ax / g0;
Ay_plot = Ay / g0;
figure
surf(Ax_plot, Ay_plot, V, V, 'EdgeColor', 'none', 'FaceColor', 'interp')
grid on
xlabel('Longitudinal accel (g)')
ylabel('Lateral accel (g)')
zlabel('Speed (m/s)')
title('g-g-v Surface from Vehicle Data')
view(3)
cb = colorbar;
ylabel(cb, 'Speed (m/s)')
hold on
for k = 1:numel(speedRings)
    plot3(Ax_plot(k,:), Ay_plot(k,:), V(k,:), 'k-')
end
hold off

%% Oval track
trackCenter = [0 0];   % [xc yc]
a = 60;               % semi-major axis in x (m)
b = 40;                % semi-minor axis in y (m)
nPts = 200;

theta = linspace(pi/2, 2*pi + pi/2, nPts + 1);
theta(end) = [];   % do not duplicate the start/end point

useOval = false;      % true = synthetic oval above, false = Sepang data
trackSpacing = 0.5;    % resample the logged track every N metres. Fixed distance spacing keeps curvature
                       % consistent at all speeds: wider spacing opens corners up, raw 50 Hz samples
                       % (0.3 m apart in slow corners) are noisy because coords are stored to ~3 mm
closeBlend = 30;       % metres over which the end of the lap is blended into the start (lap start/end
                       % lines differ by ~1.8 m sideways and ~9 deg, which otherwise makes a fake hairpin)
curvSmoothPts = 1;     % curvature smoothing window in points (1 = none)

if useOval
    xTrack = trackCenter(1) + a*cos(theta);
    yTrack = trackCenter(2) + b*sin(theta);
    trackName = 'Oval Track';
else
    % Find Data\Sepang.mat. mfilename points to a temp copy when using Run Section,
    % so also check the file open in the editor, the MATLAB path, and the current folder.
    searchDirs = {fileparts(mfilename('fullpath'))};
    try searchDirs{end+1} = fileparts(matlab.desktop.editor.getActiveFilename); catch, end
    searchDirs{end+1} = fileparts(which('Solver_Script'));
    searchDirs{end+1} = pwd;
    dataFile = '';
    for k = 1:numel(searchDirs)
        f = fullfile(searchDirs{k}, 'Data', 'UGR26_FSUK_Ziru_Lap3.mat');
        if ~isempty(searchDirs{k}) && isfile(f), dataFile = f; break, end
    end
    if isempty(dataFile)
        error('Could not find Data\Sepang.mat. Change folder to the LTS folder (cd) and try again.');
    end
    trackData = load(dataFile, 'Car_Coord_X', 'Car_Coord_Y');
    xRaw = trackData.Car_Coord_X.Value;
    yRaw = trackData.Car_Coord_Y.Value;

    % Resample at a fixed distance spacing
    sRaw = [0, cumsum(hypot(diff(xRaw), diff(yRaw)))];
    [sRaw, rawIdx] = unique(sRaw);              % drop repeated points (car stationary)
    sTrackRaw = 0:trackSpacing:sRaw(end);       % logged distance of each track point (used for comparison)
    xTrack = interp1(sRaw, xRaw(rawIdx), sTrackRaw);
    yTrack = interp1(sRaw, yRaw(rawIdx), sTrackRaw);

    % Close the loop smoothly: blend the last closeBlend metres onto the start line's tangent
    Lc = sTrackRaw(end) + trackSpacing;
    j  = find(sTrackRaw >= 3, 1);
    h  = [xTrack(j) - xTrack(1), yTrack(j) - yTrack(1)];
    h  = h / norm(h);
    w  = min(max((sTrackRaw - (sTrackRaw(end) - closeBlend)) / closeBlend, 0), 1);
    w  = 0.5 * (1 - cos(pi * w));
    xTrack = (1 - w) .* xTrack + w .* (xTrack(1) - (Lc - sTrackRaw) * h(1));
    yTrack = (1 - w) .* yTrack + w .* (yTrack(1) - (Lc - sTrackRaw) * h(2));
    trackName = 'Sepang';
end

figure
plot(xTrack, yTrack, '-', 'LineWidth', 2)
axis equal
grid on
xlabel('X (m)')
ylabel('Y (m)')
title(trackName)

sLap = [0, cumsum(hypot(diff(xTrack), diff(yTrack)))];
L = sLap(end) + hypot(xTrack(1) - xTrack(end), yTrack(1) - yTrack(end));

xPrev = circshift(xTrack, 1);
xNext = circshift(xTrack, -1);
yPrev = circshift(yTrack, 1);
yNext = circshift(yTrack, -1);

sPrev = circshift(sLap, 1);
sNext = circshift(sLap, -1);
sPrev(1) = sLap(end) - L;
sNext(end) = sLap(1) + L;

dx = (xNext - xPrev) ./ (sNext - sPrev);
dy = (yNext - yPrev) ./ (sNext - sPrev);

dxPrev = circshift(dx, 1);
dxNext = circshift(dx, -1);
dyPrev = circshift(dy, 1);
dyNext = circshift(dy, -1);

ddx = (dxNext - dxPrev) ./ (sNext - sPrev);
ddy = (dyNext - dyPrev) ./ (sNext - sPrev);

kappa = abs(dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^(3/2);
kappa = smoothdata(kappa, "movmean", curvSmoothPts);
kappa = max(kappa, eps);
radius = 1 ./ max(kappa, eps);
radius = min(radius, 500);
radius(~isfinite(radius)) = NaN;




figure
plot(sLap, radius, '-', 'LineWidth', 2)
grid on
xlabel('Distance along track (m)')
ylabel('Radius of Curvature (m)')
title('Radius of Curvature')

figure
plot(sLap, gradient(radius, sLap), '-', 'LineWidth', 2)
grid on
xlabel('Distance along track (m)')
ylabel('dRadius/ds')
title('Rate of Change of Radius')

% Define Apex

dR = gradient(radius, sLap);
dRR = gradient(dR, sLap);

[apex_radius, apex_sLaps] = findpeaks(-radius, sLap, "MinPeakProminence", 40, "MinPeakHeight", -400);
apex_radius = -apex_radius;

figure
plot(sLap, radius, '-', 'LineWidth', 2)
hold on

plot(apex_sLaps, apex_radius, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8)

grid on
xlabel('Distance along track (m)')
ylabel('Radius of Curvature (m)')
title('Radius of Curvature')

%% Solve for peak Ay at apex

vGrid = linspace(0, max(speedRings), 100);   % stay within GGV

ayEnvelope = max(Ay, [], 2); 
ayAvailGrid = interp1(speedRings(:), ayEnvelope(:), vGrid, 'pchip', 'extrap');

vMaxAtApex = NaN(size(apex_radius));

for k = 1:numel(apex_sLaps)
    turnRadius = apex_radius(k);

    ayReq = vGrid.^2 ./ turnRadius;

    idx = find(ayReq <= ayAvailGrid, 1, 'last');

    if ~isempty(idx)
        vMaxAtApex(k) = vGrid(idx);
    end
end


%% Cornering speed limit at every track point
% Highest speed where the required lateral accel (v^2/R) fits inside the GGV envelope.
% Used to cap the accel/brake passes so they never run out of grip and leave NaN gaps.
vFine = linspace(0, max(speedRings), 1000);
ayAvailFine = interp1(speedRings(:), ayEnvelope(:), vFine, 'pchip');
feasibleV = (vFine.^2) ./ radius(:) <= ayAvailFine;          % numel(sLap) x numel(vFine)
[~, lastIdx] = max(feasibleV .* (1:numel(vFine)), [], 2);
vLimit = vFine(lastIdx);                                      % 1 x numel(sLap)

% Solve Acceleration

function [ax, ay] = ax_solver(vel, inst_radius, isAccel, ggvDv, AxTab, AyTab)
% GGV slice at this speed, read from the precomputed tables (linear blend between the two
% nearest 0.01 m/s rows - matches interp1(...,'pchip') on speedRings to ~1e-6 m/s)
f = vel / ggvDv;
i = min(floor(f) + 1, size(AxTab, 1) - 1);
w = f - (i - 1);
axAtVel = AxTab(i, :) * (1 - w) + AxTab(i + 1, :) * w;
ayAtVel = AyTab(i, :) * (1 - w) + AyTab(i + 1, :) * w;

% Keep only points that satisfy the lateral requirement
feasible = ayAtVel >= vel.^2 ./ inst_radius;

ax = NaN;
ay = NaN;

if any(feasible)
    feasibleIdx = find(feasible);
    if isAccel
        [ax, localIdx] = max(axAtVel(feasibleIdx));
    else
        [ax, localIdx] = min(axAtVel(feasibleIdx));
    end
    idx = feasibleIdx(localIdx);
    ay = ayAtVel(idx);
end
end


%% Precompute GGV slices on a fine speed grid (replaces interp1 inside the solver loop)
ggvDv = 0.01;                                                  % m/s
vTab  = (0:ggvDv:max(speedRings) + ggvDv)';
AxTab = interp1(speedRings(:), Ax, vTab, 'pchip', 'extrap');   % numel(vTab) x numel(phi)
AyTab = interp1(speedRings(:), Ay, vTab, 'pchip', 'extrap');

% Solve for vCar
solveTimer = tic;

vCar = inf(1, numel(sLap));
ayCar = NaN(1, numel(sLap));

for i = 1:numel(apex_sLaps)
    apex_sLap = apex_sLaps(i);
    [~, apex_idx] = min(abs(sLap - apex_sLap));

    candidate = inf(1, numel(sLap));
    candidateAy = NaN(1, numel(sLap));

    candidate(apex_idx) = vMaxAtApex(i);
    candidateAy(apex_idx) = candidate(apex_idx).^2 ./ apex_radius(i);

    for n = apex_idx + 1:numel(sLap)
        vel = candidate(n - 1);
        if ~isfinite(vel)
            break
        end

        inst_radius = radius(n);
        [ax, ay] = ax_solver(vel, inst_radius, true, ggvDv, AxTab, AyTab);
        ds = sLap(n) - sLap(n - 1);

        if isnan(ax)
            vNew = vLimit(n);                 % out of grip: car sits at its cornering limit
        else
            vNew = sqrt(max(0, vel.^2 + 2 * ax * ds));
        end

        if isnan(ax) || vNew > vLimit(n)
            vNew = vLimit(n);
            ay = vNew.^2 ./ inst_radius;
        end

        candidate(n) = vNew;
        candidateAy(n) = ay;

        % Early stop: an earlier apex's acceleration pass is already slower here. Acceleration
        % passes never cross back (a faster entry speed always gives a faster exit speed), so
        % this pass can't lower vCar any further. Only safe for acceleration passes - braking
        % passes can cross a forward pass where it snaps down to vLimit, so they always run in full.
        if vNew > vCar(n)
            break
        end
    end

    for n = apex_idx - 1:-1:1
        vel = candidate(n + 1);
        if ~isfinite(vel)
            break
        end

        inst_radius = radius(n);
        [ax, ay] = ax_solver(vel, inst_radius, false, ggvDv, AxTab, AyTab);
        ds = sLap(n + 1) - sLap(n);

        if isnan(ax)
            vNew = vLimit(n);                 % out of grip: car sits at its cornering limit
        else
            vNew = sqrt(max(0, vel.^2 + 2 * (-ax) * ds));
        end

        if isnan(ax) || vNew > vLimit(n)
            vNew = vLimit(n);
            ay = vNew.^2 ./ inst_radius;
        end

        candidate(n) = vNew;
        candidateAy(n) = ay;
    end

    finiteMask = isfinite(candidate);
    betterMask = finiteMask & candidate < vCar;
    vCar(betterMask) = candidate(betterMask);
    ayCar(betterMask) = candidateAy(betterMask);
end

% Wrap-around solve: start at sLap(end) and accelerate through sLap(1)
firstApexS = min(apex_sLaps);
[~, firstApexIdx] = min(abs(sLap - firstApexS));

if isfinite(vCar(end))
    startCandidate = inf(1, numel(sLap));
    startCandidateAy = NaN(1, numel(sLap));
    prevVel = vCar(end);

    for n = 1:firstApexIdx
        inst_radius = radius(n);
        [ax, ay] = ax_solver(prevVel, inst_radius, true, ggvDv, AxTab, AyTab);

        if n == 1
            ds = L - sLap(end);
        else
            ds = sLap(n) - sLap(n - 1);
        end

        if isnan(ax)
            vNew = vLimit(n);                 % out of grip: car sits at its cornering limit
        else
            vNew = sqrt(max(0, prevVel.^2 + 2 * ax * ds));
        end

        if isnan(ax) || vNew > vLimit(n)
            vNew = vLimit(n);
            ay = vNew.^2 ./ inst_radius;
        end

        startCandidate(n) = vNew;
        startCandidateAy(n) = ay;
        prevVel = startCandidate(n);
    end

    finiteMask = isfinite(startCandidate);
    betterMask = finiteMask & startCandidate < vCar;
    vCar(betterMask) = startCandidate(betterMask);
    ayCar(betterMask) = startCandidateAy(betterMask);
end

vCar(isinf(vCar)) = NaN;
fprintf('Speed solve: %.1f s (%d track points, %d apexes)\n', toc(solveTimer), numel(sLap), numel(apex_sLaps));
% Lateral accel actually used = v^2/R. (The solver's ay is the GGV boundary point it picked,
% which can be well above what the corner needs.)
ayCar = vCar.^2 ./ radius;

figure
plot(sLap, vCar, '-', 'LineWidth', 2)
grid on
xlabel('Distance along track (m)')
ylabel('vCar [m/s]')
title('Speed Trace')

figure
plot(sLap, ayCar / 9.81, '-', 'LineWidth', 2)
grid on
xlabel('Distance along track (m)')
ylabel('a_y [g]')
title('Lateral Acceleration Trace')

%% Comparison: simulation vs logged lap (speed, lateral g, longitudinal g)
% Logged channels are interpolated by logged distance onto the same points as xTrack/yTrack,
% so they line up with sLap point for point.
if useOval
    warning('Comparison needs the logged lap: set useOval = false.');
else
    lapData = load(dataFile, 'Ground_Speed', 'CG_Accel_Lateral', 'CG_Accel_Longitudinal');
    toTrack = @(sig) interp1(sRaw, sig(rawIdx), sTrackRaw);

    vMeas     = toTrack(lapData.Ground_Speed.Value) / 3.6;                              % km/h -> m/s
    gLatMeas  = toTrack(smoothdata(lapData.CG_Accel_Lateral.Value,      'movmean', 5)); % g; 0.1 s smoothing removes accel noise
    gLongMeas = toTrack(smoothdata(lapData.CG_Accel_Longitudinal.Value, 'movmean', 5)); % g; +ve = accelerating
    tMeas     = toTrack(lapData.Ground_Speed.Time);
    tMeas     = tMeas - tMeas(1);

    % Simulated lateral g, signed like the logger (+ve = turning left / anticlockwise in X-Y)
    turnDir  = sign(smoothdata(dx .* ddy - dy .* ddx, 'movmean', max(3, round(5/trackSpacing))));
    gLatSim  = turnDir .* ayCar / g0;

    % Simulated longitudinal g from the speed trace: a_x = d(v^2/2)/ds
    gLongSim = gradient(vCar.^2 / 2, sLap) / g0;

    % Simulated time: distance / mean speed over each segment (last segment wraps to the start)
    segLen = [diff(sLap), L - sLap(end)];
    vSeg   = (vCar + circshift(vCar, -1)) / 2;
    dtSim  = segLen ./ vSeg;
    tSim   = [0, cumsum(dtSim(1:end-1))];

    lapTimeSim  = sum(dtSim);
    lapTimeMeas = lapData.Ground_Speed.Time(end) - lapData.Ground_Speed.Time(1);

    nGaps = nnz(isnan(vCar));
    if nGaps > 0
        warning('%d track points have no simulated speed (NaN) - simulated lap time is not valid.', nGaps);
    end

    fprintf('\n                 Simulated   Logged\n');
    fprintf('Lap time (s)     %8.2f  %8.2f   (delta %+.2f s)\n', lapTimeSim, lapTimeMeas, lapTimeSim - lapTimeMeas);
    fprintf('Top speed (m/s)  %8.1f  %8.1f\n', max(vCar), max(vMeas));
    fprintf('Min speed (m/s)  %8.1f  %8.1f\n', min(vCar), min(vMeas));
    fprintf('Peak |gLat| (g)  %8.2f  %8.2f\n', max(abs(gLatSim)), max(abs(gLatMeas)));
    fprintf('Peak accel (g)   %8.2f  %8.2f\n', max(gLongSim), max(gLongMeas));
    fprintf('Peak brake (g)   %8.2f  %8.2f\n', -min(gLongSim), -min(gLongMeas));
    if max(vMeas) > max(speedRings)
        fprintf(['Note: logged speed reaches %.1f m/s but the GGV data stops at %.1f m/s,\n' ...
            '      so the simulation cannot go faster than that.\n'], max(vMeas), max(speedRings));
    end

    figure
    tl = tiledlayout(4, 1, 'TileSpacing', 'compact');
    title(tl, sprintf('Simulation vs Logged - %s  (sim %.2f s vs logged %.2f s)', ...
        trackName, lapTimeSim, lapTimeMeas))

    ax1 = nexttile;
    plot(sLap, vMeas, '-', 'LineWidth', 1.5, 'DisplayName', 'Logged (Lap 3)')
    hold on
    plot(sLap, vCar, '-', 'LineWidth', 1.5, 'DisplayName', 'Simulated')
    yline(max(speedRings), ':k', 'GGV data limit', 'HandleVisibility', 'off');
    plot(apex_sLaps, vMaxAtApex, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5, 'DisplayName', 'Apex limit')
    hold off
    grid on
    ylabel('vCar (m/s)')
    legend('Location', 'best')

    ax2 = nexttile;
    plot(sLap, gLatMeas, '-', 'LineWidth', 1.5)
    hold on
    plot(sLap, gLatSim, '-', 'LineWidth', 1.5)
    hold off
    yline(0, 'k-');
    grid on
    ylabel('gLat (g)')

    ax3 = nexttile;
    plot(sLap, gLongMeas, '-', 'LineWidth', 1.5)
    hold on
    plot(sLap, gLongSim, '-', 'LineWidth', 1.5)
    hold off
    yline(0, 'k-');
    grid on
    ylabel('gLong (g)')

    ax4 = nexttile;
    plot(sLap, tSim - tMeas, '-', 'LineWidth', 1.5, 'Color', [0.3 0.3 0.3])
    yline(0, 'k-');
    grid on
    ylabel('\Deltat sim - logged (s)')
    xlabel('Distance along track (m)')

    linkaxes([ax1 ax2 ax3 ax4], 'x')
    xlim([0 sLap(end)])
end
