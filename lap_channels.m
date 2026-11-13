function T = lap_channels(res, L)
% LAP_CHANNELS  Every lap channel (lap sim + lap_loads) as one table, one row per track point.
%   T = lap_channels(res)        lap sim channels only
%   T = lap_channels(res, L)     plus the loads / suspension channels from lap_loads
%
% Channel names, units and conversions come from Data/Channel_List.csv:
%   Channel Name, SI Unit, Engineering Unit, Desc., SI to Engineering Conversion
% Channels listed there are passed in here in SI and stored in the table in engineering
% units (data * conversion); T.Properties.VariableUnits holds the engineering unit and
% T.Properties.VariableDescriptions the description.
% Channels not (yet) in the CSV are added as-is with the unit given below.

chList = readChannelList();

c = {};   % {name, unit, desc, data}
add('sLap', res.s);
add('tLap', res.tSim, 's');
add('X', res.x, 'm');
add('Y', res.y, 'm');
add('xCornerRadius', res.radius);
add('vCar', res.vCar);                              % m/s
add('gLat', res.gLatSim * 9.81);                    % res is in g (g0 = 9.81) -> m/s^2
add('gLong', res.gLongSim * 9.81);
% if isfield(res, 'logged') && ~isempty(res.logged)
%     add('Logged speed', res.logged.v, 'm/s');
%     add('Logged lateral acceleration', res.logged.gLat, 'g');
%     add('Logged longitudinal acceleration', res.logged.gLong, 'g');
%     add('Logged time', res.logged.t, 's');
%     add('Time delta to logged', res.tSim - res.logged.t, 's');
% end

if nargin > 1 && ~isempty(L)
    cn = L.corners;
    add('xFRH', L.FRH / 1000);                      % lap_loads is in mm -> m
    add('xRRH', L.RRH / 1000);
    add('aRoll', deg2rad(L.roll));                  % lap_loads is in deg -> rad
    add('aPitch', deg2rad(L.pitch));
    % add('Front suspension roll', L.rollSuspF, 'deg');
    % add('Rear suspension roll', L.rollSuspR, 'deg');
    add('FAeroF', L.aeroF);
    add('FAeroR', L.aeroR);
    add('FAeroTotal', L.aeroF + L.aeroR);
    add('rAeroBal', L.aeroF ./ (L.aeroF + L.aeroR));
    add('CzF', L.ClF);
    add('CzR', L.ClR);
    add('CzT', L.ClF + L.ClR);
    add('Longitudinal load transfer', L.dFzLong, 'N');
    add('Front lateral load transfer', L.dFzLatF, 'N');
    add('Rear lateral load transfer', L.dFzLatR, 'N');
    add('rLoadTransfer', L.dFzLatF ./ (L.dFzLatF + L.dFzLatR));
    perCorner('FzTyre', L.Fz);
    add('FzTyreF', L.Fz(1, :) + L.Fz(2, :));
    add('FzTyreR', L.Fz(3, :) + L.Fz(4, :));
    add('Front axle load share', (L.Fz(1, :) + L.Fz(2, :)) ./ sum(L.Fz, 1), '-');
    perCorner('Wheel travel ', L.wheelTravel, 'mm');
    perCorner('Tyre deflection ', L.tyreDefl, 'mm');
    perCorner('xCornerSpring', L.springTravel / 1000);   % mm -> m
    perCorner('Shock length ', L.shockLength, 'mm');
    perCorner('FCornerSpring', L.springForce);
    perCorner('FxTyre', L.Fx);
    perCorner('FyTyre', L.Fy);
    perCorner('MBrake', L.brakeTorque);
    add('Cx', repmat(L.CdA, 1, numel(L.s)));
    for m = 1:numel(L.memberNames)
        perCorner(['F' L.memberNames{m}], L.memberForce{m});
    end
end

T = table();
for k = 1:size(c, 1)
    T.(c{k, 1}) = c{k, 4}(:);
end
T.Properties.VariableUnits = c(:, 2)';
T.Properties.VariableDescriptions = c(:, 3)';

    function add(name, data, unit)
        % Channel in the CSV: data is SI, converted to engineering units.
        % Otherwise: data is stored as-is and unit must be given.
        i = find(strcmp(chList.name, name), 1);
        data = double(data(:))';
        if ~isempty(i)
            c(end + 1, :) = {name, chList.engUnit{i}, chList.desc{i}, data * chList.factor(i)};
        elseif nargin > 2
            c(end + 1, :) = {name, unit, '', data};
        else
            error('lap_channels:unknownChannel', ...
                'Channel ''%s'' is not in Data/Channel_List.csv and no unit was given.', name);
        end
    end

    function perCorner(name, data, varargin)
        for j = 1:4
            add([name cn{j}], data(j, :), varargin{:});
        end
    end
end

function chList = readChannelList()
% Read Data/Channel_List.csv into name / engUnit / desc / factor columns.
file = fullfile(fileparts(mfilename('fullpath')), 'Data', 'Channel_List.csv');
opts = detectImportOptions(file, 'Delimiter', ',', 'Encoding', 'UTF-8', ...
    'VariableNamingRule', 'preserve');
opts = setvartype(opts, 1:4, 'char');
opts = setvartype(opts, 5, 'double');
C = readtable(file, opts);
chList.name = strtrim(C{:, 1});
chList.engUnit = strtrim(C{:, 3});
chList.desc = strtrim(C{:, 4});
chList.factor = C{:, 5};
end
