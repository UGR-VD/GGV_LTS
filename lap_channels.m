function T = lap_channels(res, L)
% LAP_CHANNELS  Every lap channel (lap sim + lap_loads) as one table, one row per track point.
%   T = lap_channels(res)        lap sim channels only
%   T = lap_channels(res, L)     plus the loads / suspension channels from lap_loads
%
% Variable names are display names (e.g. 'Tyre load FL'); units are in T.Properties.VariableUnits.
% Logged channels are included when the track file had a logged lap.

c = {};   % {name, unit, data}
add('sLap', 'm', res.s);
add('tLap', 's', res.tSim);
add('X', 'm', res.x);
add('Y', 'm', res.y);
add('Corner radius', 'm', res.radius);
add('vCar', 'm/s', res.vCar);
add('vCar [kph]', 'kph', res.vCar * 3.6);
add('gLat', 'g', res.gLatSim);
add('gLong', 'g', res.gLongSim);
% if isfield(res, 'logged') && ~isempty(res.logged)
%     add('Logged speed', 'm/s', res.logged.v);
%     add('Logged lateral acceleration', 'g', res.logged.gLat);
%     add('Logged longitudinal acceleration', 'g', res.logged.gLong);
%     add('Logged time', 's', res.logged.t);
%     add('Time delta to logged', 's', res.tSim - res.logged.t);
% end

if nargin > 1 && ~isempty(L)
    cn = L.corners;
    add('xFRH', 'mm', L.FRH);
    add('xRRH', 'mm', L.RRH);
    add('aRoll', 'deg', L.roll);
    add('aPitch', 'deg', L.pitch);
    % add('Front suspension roll', 'deg', L.rollSuspF);
    % add('Rear suspension roll', 'deg', L.rollSuspR);
    add('FAeroF', 'N', L.aeroF);
    add('FAeroR', 'N', L.aeroR);
    add('FAeroTotal', 'N', L.aeroF + L.aeroR);
    add('rAeroBal', '-', L.aeroF ./ (L.aeroF + L.aeroR));
    add('ClF', '-', L.ClF);
    add('ClR', '-', L.ClR);
    add('Longitudinal load transfer', 'N', L.dFzLong);
    add('Front lateral load transfer', 'N', L.dFzLatF);
    add('Rear lateral load transfer', 'N', L.dFzLatR);
    perCorner('Tyre load', 'N', L.Fz);
    add('Front axle load', 'N', L.Fz(1, :) + L.Fz(2, :));
    add('Rear axle load', 'N', L.Fz(3, :) + L.Fz(4, :));
    add('Front axle load share', '-', (L.Fz(1, :) + L.Fz(2, :)) ./ sum(L.Fz, 1));
    perCorner('Wheel travel', 'mm', L.wheelTravel);
    perCorner('Tyre deflection', 'mm', L.tyreDefl);
    perCorner('Spring travel', 'mm', L.springTravel);
    perCorner('Shock length', 'mm', L.shockLength);
    perCorner('Spring force', 'N', L.springForce);
end

T = table();
for k = 1:size(c, 1)
    T.(c{k, 1}) = c{k, 3}(:);
end
T.Properties.VariableUnits = c(:, 2)';

    function add(name, unit, data)
        c(end + 1, :) = {name, unit, double(data(:))'};
    end

    function perCorner(name, unit, data)
        for j = 1:4
            add([name ' ' cn{j}], unit, data(j, :));
        end
    end
end
