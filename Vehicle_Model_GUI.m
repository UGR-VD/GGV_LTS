function Vehicle_Model_GUI()
% VEHICLE_MODEL_GUI  Edit the vehicle model inputs and see the suspension geometry update.
% Only the spreadsheet's input cells (orange) are editable. Calculated hardpoint coordinates
% are shown greyed out in the hardpoint tables. Needs vehicle_model_defaults.m,
% vehicle_model_calc.m, vehicle_ggv.m, vehicle_powertrain.m and lap_sim.m on the path.
% The Lap Sim tab runs lap_sim on a track file (default Data\UGR26_FSUK_Ziru_Lap3.mat).
% Requires MATLAB R2020b or newer.

inp = vehicle_model_defaults();
[veh, calcCoords] = vehicle_model_calc(inp);

%% Input definitions (field, label, unit, options for dropdowns)
scalarTabs = {
    'Mass & Dimensions', {
        'm_vehicle',                  'Vehicle mass',                       'kg',    [];
        'm_driver',                   'Driver mass',                        'kg',    [];
        'r_vehicle_front_rear_mass',  'Vehicle weight distribution (front)', '-',    [];
        'h_vehicle_cog',              'Vehicle CoG height',                 'mm',    [];
        'h_driver_cog',               'Driver CoG height',                  'mm',    [];
        'r_driver_front_rear_mass',   'Driver F/R split (front)',           '-',     [];
        'm_unsprungF',                'Front unsprung mass',                'kg',    [];
        'm_unsprungR',                'Rear unsprung mass',                 'kg',    [];
        'x_wheelbase',                'Wheelbase',                          'mm',    [];
        'x_trackF',                   'Front track width',                  'mm',    [];
        'x_trackR',                   'Rear track width',                   'mm',    []};
    'Springs & Dampers', {
        'k_springF_imperial',         'Front spring rate',                  'lb/in', [];
        'k_springR_imperial',         'Rear spring rate',                   'lb/in', [];
        'r_MR_lowF',                  'Front low motion ratio',             '-',     [];
        'r_MR_highF',                 'Front high motion ratio',            '-',     [];
        'MR_settingF',                'Front motion ratio setting',         '',      {'High', 'Low'};
        'r_MR_lowR',                  'Rear low motion ratio',              '-',     [];
        'r_MR_highR',                 'Rear high motion ratio',             '-',     [];
        'MR_settingR',                'Rear motion ratio setting',          '',      {'High', 'Low'};
        'x_shock_extended',           'Shock fully extended length',        'mm',    [];
        'x_shock_compressed',         'Shock fully compressed length',      'mm',    []};
    'Roll & Heave Springs', {
        'ARB_settingF',               'Front ARB setting',                  '',      {'Detached', 'Soft', 'Medium', 'Hard'};
        'ARB_materialF',              'Front ARB material',                 '',      {'T45', '4130', 'Ti-6Al-4V', '7075-T6'};
        'x_ARB_arm_softF',            'Front ARB arm (soft)',               'mm',    [];
        'x_ARB_arm_mediumF',          'Front ARB arm (medium)',             'mm',    [];
        'x_ARB_arm_hardF',            'Front ARB arm (hard)',               'mm',    [];
        'x_ARB_ODF',                  'Front ARB outer diameter',           'mm',    [];
        'x_ARB_IDF',                  'Front ARB inner diameter',           'mm',    [];
        'ARB_settingR',               'Rear ARB setting',                   '',      {'Detached', 'Soft', 'Medium', 'Hard'};
        'ARB_materialR',              'Rear ARB material',                  '',      {'T45', '4130', 'Ti-6Al-4V', '7075-T6'};
        'r_ARB_MR_lowR',              'Rear ARB motion ratio (low)',        '-',     [];
        'r_ARB_MR_highR',             'Rear ARB motion ratio (high)',       '-',     [];
        'x_ARB_arm_softR',            'Rear ARB arm (soft)',                'mm',    [];
        'x_ARB_arm_mediumR',          'Rear ARB arm (medium)',              'mm',    [];
        'x_ARB_arm_hardR',            'Rear ARB arm (hard)',                'mm',    [];
        'x_ARB_ODR',                  'Rear ARB outer diameter',            'mm',    [];
        'x_ARB_IDR',                  'Rear ARB inner diameter',            'mm',    [];
        'heave_settingF',             'Front heave spring setting',         '',      {'Detached', 'Soft', 'Medium', 'Hard'};
        'heave_materialF',            'Front heave spring material',        '',      {'T45', '4130', 'Ti-6Al-4V', '7075-T6'};
        'r_heave_MR_soft_lowF',       'Front heave MR soft (low)',          '-',     [];
        'r_heave_MR_soft_highF',      'Front heave MR soft (high)',         '-',     [];
        'r_heave_MR_medium_lowF',     'Front heave MR medium (low)',        '-',     [];
        'r_heave_MR_medium_highF',    'Front heave MR medium (high)',       '-',     [];
        'r_heave_MR_hard_lowF',       'Front heave MR hard (low)',          '-',     [];
        'r_heave_MR_hard_highF',      'Front heave MR hard (high)',         '-',     [];
        'x_heave_lengthF',            'Front heave spring length',          'mm',    [];
        'x_heave_armF',               'Front heave moment arm',             'mm',    [];
        'x_heave_ODF',                'Front heave outer diameter',         'mm',    [];
        'x_heave_IDF',                'Front heave inner diameter',         'mm',    [];
        'heave_settingR',             'Rear heave spring setting',          '',      {'Detached', 'Soft', 'Medium', 'Hard'};
        'heave_materialR',            'Rear heave spring material',         '',      {'T45', '4130', 'Ti-6Al-4V', '7075-T6'};
        'x_heave_lengthR',            'Rear heave spring length',           'mm',    [];
        'x_heave_armR',               'Rear heave moment arm',              'mm',    [];
        'x_heave_ODR',                'Rear heave outer diameter',          'mm',    [];
        'x_heave_IDR',                'Rear heave inner diameter',          'mm',    []};
    'Tyres & Brakes', {
        'k_TyreF_imperial',           'Front tyre spring rate',             'lb/in', [];
        'k_TyreR_imperial',           'Rear tyre spring rate',              'lb/in', [];
        'x_Tyre_diameterF_imperial',  'Front tyre outer diameter',          'in',    [];
        'x_Tyre_diameterR_imperial',  'Rear tyre outer diameter',           'in',    [];
        'x_Tyre_widthF',              'Front tyre width',                   'mm',    [];
        'x_Tyre_widthR',              'Rear tyre width',                    'mm',    [];
        'x_rim_offset',               'Rim offset',                         'mm',    [];
        'r_mu_lat_gradient',          'Lateral load sensitivity gradient',  '1/N',   [];
        'r_mu_lat_0',                 'Lateral friction coef @ 0 N',        '-',     [];
        'r_mu_lat_scale',             'Lateral scaling factor',             '-',     [];
        'r_mu_long_gradient',         'Longitudinal load sens. gradient',   '1/N',   [];
        'r_mu_long_0',                'Longitudinal friction coef @ 0 N',   '-',     [];
        'r_mu_long_scale',            'Longitudinal scaling factor',        '-',     [];
        'x_skidpad_radius',           'Skid pad radius',                    'm',     [];
        'r_brake_splitF',             'Brake split front',                  '-',     [];
        'brake_typeR',                'Rear brake type',                    '',      {'Outboard', 'Inboard'}};
    'Aero', {
        'h_static_FRH',               'Static front ride height',           'mm',    [];
        'h_static_RRH',               'Static rear ride height',            'mm',    [];
        'rho_air',                    'Air density',                        'kg/m3', [];
        'x_aero_area',                'Aero reference area',                'm2',    [];
        'aeromap_ClF(1)',           'Aeromap ClF c0',                  '-',     [];
        'aeromap_ClF(2)',            'Aeromap ClF c1',                  '-',     [];
        'aeromap_ClF(3)',            'Aeromap ClF c2',                  '-',     [];
        'aeromap_ClF(4)',            'Aeromap ClF c3',                  '-',     [];
        'aeromap_ClF(5)',            'Aeromap ClF c4',                  '-',     [];
        'aeromap_ClF(6)',            'Aeromap ClF c5',                  '-',     [];
        'aeromap_ClF(7)',            'Aeromap ClF c6',                  '-',     [];
        'aeromap_ClF(8)',            'Aeromap ClF c7',                  '-',     [];
        'aeromap_ClF(9)',            'Aeromap ClF c8',                  '-',     [];
        'aeromap_ClF(10)',            'Aeromap ClF c9',                  '-',     [];
        'aeromap_ClR(1)',            'Aeromap ClR c0',                  '-',     [];
        'aeromap_ClR(2)',            'Aeromap ClR c1',                  '-',     [];
        'aeromap_ClR(3)',            'Aeromap ClR c2',                  '-',     [];
        'aeromap_ClR(4)',            'Aeromap ClR c3',                  '-',     [];
        'aeromap_ClR(5)',            'Aeromap ClR c4',                  '-',     [];
        'aeromap_ClR(6)',            'Aeromap ClR c5',                  '-',     [];
        'aeromap_ClR(7)',            'Aeromap ClR c6',                  '-',     [];
        'aeromap_ClR(8)',            'Aeromap ClR c7',                  '-',     [];
        'aeromap_ClR(9)',            'Aeromap ClR c8',                  '-',     [];
        'aeromap_ClR(10)',            'Aeromap ClR c9',                  '-',     []};
    'Powertrain', {
        'drive_type',                 'Drive type',                         '',      {'RWD', 'AWD'};
        'cell_type',                 'Cell type',                          '',      {'VTC6', 'P50B'};
        'n_cells_series',             'Cells in series (per segment)',      '-',     [];
        'n_cells_parallel',           'Cells in parallel',                  '-',     [];
        'n_segments',                 'Segments',                           '-',     [];
        'I_burst_limit',              'Max burst current (limited)',        'A',     [];
        'motor',                      'Motor',                              '',      {'188 HV', '188 MV', '188 LV', '208 HV', '208 MV', '208 LV', '228 HV', '228 MV', '228 LV'};
        'R_internal',                 'Internal resistance',                'Ohm',   [];
        'P_accumulator_max',          'Max accumulator power',              'kW',    [];
        'r_planetary',                'Planetary gearbox ratio',            '-',     [];
        'r_final_drive',              'Final drive',                        '-',     [];
        'r_gears(1)',                 'Gear ratio 1',                       '-',     [];
        'r_gears(2)',                 'Gear ratio 2',                       '-',     [];
        'r_gears(3)',                 'Gear ratio 3',                       '-',     [];
        'r_gears(4)',                 'Gear ratio 4',                       '-',     [];
        'eff_accumulator',            'Accumulator efficiency',             '-',     [];
        'eff_inverter',               'Inverter efficiency',                '-',     [];
        'eff_motor',                  'Motor efficiency',                   '-',     [];
        'eff_gears',                  'Gears efficiency',                   '-',     [];
        'eff_gearbox',                'Gearbox efficiency',                 '-',     [];
        'eff_other_drivetrain',       'Other drivetrain efficiency',        '-',     [];
        'Nm_to_kW',                   'Nm to kW (rpm) conversion',          '-',     [];
        'dt_accel',                   'Acceleration sim time step',         's',     [];
        'x_accel_length',             'Acceleration length',                'm',     [];
        'r_rolling_resistance',       'Rolling resistance',                 '-',     []}};

pointDefs = {
    'upr_upright',      'Upper upright';
    'lwr_upright',      'Lower upright';
    'uprfwd_outbrd',    'Upper chassis front';
    'uprrwd_outbrd',    'Upper chassis rear';
    'lwrfwd_outbrd',    'Lower chassis front';
    'lwrrwd_outbrd',    'Lower chassis rear';
    'toe_outbrd',       'Toe link outboard';
    'toe_inbrd',        'Toe link inboard';
    'prod_outbrd',      'Pushrod to upright';
    'prod_inbrd',       'Pushrod to rocker MR1';
    'prod_inbrd_MR2',   'Pushrod to rocker MR2';
    'rocker_centre',    'Rocker centre';
    'rocker_spring',    'Spring to rocker';
    'spring_chassis',   'Spring to chassis';
    'arb_upr_droplink', 'Upper ARB droplink';
    'arb_lwr_droplink', 'Lower ARB droplink';
    'arb_centre',       'ARB centre'};

% Colours
cWish   = [0.00 0.45 0.74];
cUpr    = [0.15 0.15 0.15];
cToe    = [0.47 0.67 0.19];
cProd   = [0.85 0.33 0.10];
cRocker = [0.60 0.60 0.60];
cSpring = [0.80 0.10 0.10];
cARB    = [0.49 0.18 0.56];
cTyre   = [0.30 0.30 0.30];
cIC     = [0.93 0.69 0.13];

%% Layout
fig = uifigure('Name', 'Vehicle Model', 'Position', [60 60 1500 880]);
main = uigridlayout(fig, [1 2]);
main.ColumnWidth = {480, '1x'};

left = uigridlayout(main, [3 1]);
left.RowHeight = {'1x', 30, 40};
left.Padding = [0 0 0 0];
inTabs = uitabgroup(left);

btnRow = uigridlayout(left, [1 2]);
btnRow.Padding = [0 0 0 0];
uibutton(btnRow, 'Text', 'Reset to spreadsheet values', 'ButtonPushedFcn', @(~, ~) onReset());
uibutton(btnRow, 'Text', 'Send inp / veh to workspace', 'ButtonPushedFcn', @(~, ~) onExport());

status = uilabel(left, 'Text', '', 'WordWrap', 'on');

right = uigridlayout(main, [3 1]);
right.RowHeight = {'2x', 24, '1x'};
right.RowSpacing = 4;
plotTabs = uitabgroup(right);
plotTabs.Layout.Row = 1;
ax3 = makeAxes(plotTabs, '3D');
axFF = makeAxes(plotTabs, 'Front view - front axle');
axFR = makeAxes(plotTabs, 'Front view - rear axle');
axS = makeAxes(plotTabs, 'Side view');
tabGGV = uitab(plotTabs, 'Title', 'GGV');
gGGV = uigridlayout(tabGGV, [1 2]);
gGGV.ColumnWidth = {'1x', 430};
axGGV = uiaxes(gGGV);
ggvTable = uitable(gGGV, 'ColumnName', {'v (m/s)', 'Lat (g)', 'Accel grip (g)', 'Brake (g)', 'Powertrain (g)', 'Balance', 'Brake bias'}, ...
    'RowName', {}, 'ColumnEditable', false);
axPT = makeAxes(plotTabs, 'Powertrain');

% Lap sim tab
tabLap = uitab(plotTabs, 'Title', 'Lap Sim');
gLap = uigridlayout(tabLap, [2 1]);
gLap.RowHeight = {30, '1x'};
gLapCtrl = uigridlayout(gLap, [1 10]);
gLapCtrl.Padding = [0 0 0 0];
gLapCtrl.ColumnWidth = {60, '1x', 70, 85, 110, 105, 125, 85, 95, 300};
uilabel(gLapCtrl, 'Text', 'Track file');
trackField = uieditfield(gLapCtrl, 'text', 'Value', defaultTrackFile());
uibutton(gLapCtrl, 'Text', 'Browse...', 'ButtonPushedFcn', @(~, ~) onBrowse());
uilabel(gLapCtrl, 'Text', 'Compare with', 'HorizontalAlignment', 'right');
ddCompare = uidropdown(gLapCtrl, 'Items', {'Logged lap', 'Baseline sim', 'None'}, 'Value', 'Logged lap', ...
    'ValueChangedFcn', @(~, ~) drawLap());
uibutton(gLapCtrl, 'Text', 'Set as baseline', 'ButtonPushedFcn', @(~, ~) onSetBaseline());
cbAutoLap = uicheckbox(gLapCtrl, 'Text', 'Re-run on changes', 'Value', false);
uibutton(gLapCtrl, 'Text', 'Run lap sim', 'ButtonPushedFcn', @(~, ~) runLap());
uibutton(gLapCtrl, 'Text', 'Clear baseline', 'ButtonPushedFcn', @(~, ~) onClearBaseline());
lapLabel = uilabel(gLapCtrl, 'Text', '', 'FontWeight', 'bold');
gLapPlots = uigridlayout(gLap, [1 2]);
gLapPlots.ColumnWidth = {'1x', '2x'};
gLapPlots.Padding = [0 0 0 0];
axTrack = uiaxes(gLapPlots);
gLapTraces = uigridlayout(gLapPlots, [4 1]);
gLapTraces.Padding = [0 0 0 0];
gLapTraces.RowSpacing = 2;
axLapV = uiaxes(gLapTraces);
axLapLat = uiaxes(gLapTraces);
axLapLong = uiaxes(gLapTraces);
axLapDt = uiaxes(gLapTraces);
lapRes = [];
lapInp = [];      % inputs used for lapRes
baseRes = [];     % baseline lap sim (the first run, or the one picked with "Set as baseline")
baseInp = [];
baseFile = '';
lapLoads = [];    % lap_loads result for lapRes (loads, ride heights, suspension travel around the lap)
lapCh = [];       % lap_channels table for lapRes + lapLoads (every channel, one row per track point)
baseCh = [];      % same for the baseline run
lapShockLim = []; % [fully compressed, fully extended] shock length for lapRes (mm)
loadsShowBase = false;
panelSel = {{'FzTyreFL', 'FzTyreFR', 'FzTyreRL', 'FzTyreRR'}, ...
    {'xFRH', 'xRRH'}, {'aRoll', 'aPitch'}, ...
    {'Shock length FL', 'Shock length FR', 'Shock length RL', 'Shock length RR'}};

% Lap channels tab: every lap sim / lap loads channel, user picks what to plot, and export
tabLoads = uitab(plotTabs, 'Title', 'Lap Channels');
gLoads = uigridlayout(tabLoads, [1 2]);
gLoads.ColumnWidth = {'1x', 330};
gLoads.Padding = [4 4 4 4];
gLoadTraces = uigridlayout(gLoads, [4 1]);
gLoadTraces.Padding = [0 0 0 0];
gLoadTraces.RowSpacing = 2;
axLd = gobjects(1, 4);
for k = 1:4, axLd(k) = uiaxes(gLoadTraces); end
gLoadSide = uigridlayout(gLoads, [6 1]);
gLoadSide.RowHeight = {24, 24, '1.3x', 26, '1x', 54};
gLoadSide.Padding = [0 0 0 0];
gLoadSide.RowSpacing = 4;
gX = uigridlayout(gLoadSide, [1 2]);
gX.Padding = [0 0 0 0];
gX.ColumnWidth = {70, '1x'};
uilabel(gX, 'Text', 'X axis');
ddX = uidropdown(gX, 'Items', {'sLap'}, 'Value', 'sLap', 'ValueChangedFcn', @(~, ~) drawLoads());
gPanel = uigridlayout(gLoadSide, [1 2]);
gPanel.Padding = [0 0 0 0];
gPanel.ColumnWidth = {70, '1x'};
uilabel(gPanel, 'Text', 'Y channels');
ddPanel = uidropdown(gPanel, 'Items', {'Plot 1 (top)', 'Plot 2', 'Plot 3', 'Plot 4 (bottom)'}, ...
    'ValueChangedFcn', @(~, ~) onPanelPick());
lbCh = uilistbox(gLoadSide, 'Items', {}, 'Multiselect', 'on', 'ValueChangedFcn', @(~, ~) onChannelPick(), ...
    'Tooltip', 'Ctrl / Shift + click to pick several channels for the selected plot; pick none to hide it');
gExport = uigridlayout(gLoadSide, [1 2]);
gExport.Padding = [0 0 0 0];
gExport.ColumnWidth = {'1x', 130};
uibutton(gExport, 'Text', 'Export channels...', 'ButtonPushedFcn', @(~, ~) onExportChannels());
cbExportPlotted = uicheckbox(gExport, 'Text', 'Plotted only', 'Value', false);
loadsTable = uitable(gLoadSide, 'ColumnName', {'Channel', 'Min', 'Max', 'Unit'}, 'RowName', {}, ...
    'ColumnEditable', false, 'ColumnWidth', {140, 60, 60, 45});
loadsNote = uilabel(gLoadSide, 'Text', '', 'WordWrap', 'on', 'VerticalAlignment', 'top');
view(ax3, -37.5, 30);

btnOutToggle = uibutton(right, 'Text', [char(9662) ' Hide results table'], ...
    'HorizontalAlignment', 'left', 'ButtonPushedFcn', @(~, ~) onToggleOutputs());
btnOutToggle.Layout.Row = 2;
outTable = uitable(right, 'ColumnName', {'Output', 'Front', 'Rear', 'Unit'}, 'RowName', {}, ...
    'ColumnEditable', false, 'ColumnWidth', {260, 'auto', 'auto', 80});
outTable.Layout.Row = 3;

%% Scalar input tabs
scalarFields = {};
scalarHandles = {};
ptGrid = [];
for t = 1:size(scalarTabs, 1)
    items = scalarTabs{t, 2};
    tab = uitab(inTabs, 'Title', scalarTabs{t, 1});
    g = uigridlayout(tab, [size(items, 1) 3]);
    g.ColumnWidth = {'1x', 120, 50};
    g.RowHeight = repmat({24}, 1, size(items, 1));
    g.Scrollable = 'on';
    for k = 1:size(items, 1)
        f = items{k, 1};
        uilabel(g, 'Text', items{k, 2});
        if isempty(items{k, 4})
            h = uieditfield(g, 'numeric', 'Value', getInp(inp, f), 'ValueDisplayFormat', '%.6g', ...
                'ValueChangedFcn', @(src, ~) onScalar(f, src.Value));
        else
            h = uidropdown(g, 'Items', items{k, 4}, 'Value', char(getInp(inp, f)), ...
                'ValueChangedFcn', @(src, ~) onScalar(f, src.Value));
        end
        scalarFields{end + 1} = f; %#ok<AGROW>
        scalarHandles{end + 1} = h; %#ok<AGROW>
        uilabel(g, 'Text', items{k, 3});
    end
    if strcmp(scalarTabs{t, 1}, 'Powertrain'), ptGrid = g; end
end

%% Torque curve override (bottom of the Powertrain tab)
nPt = numel(ptGrid.RowHeight);
ptGrid.RowHeight = [ptGrid.RowHeight, {16, 28, 24, 26, 220, 44}];
placeRow(uilabel(ptGrid, 'Text', ''), nPt + 1);
placeRow(uilabel(ptGrid, 'Text', 'Motor torque curve override', 'FontWeight', 'bold'), nPt + 2);
cbTorqueOverride = uicheckbox(ptGrid, 'Text', 'Use my own rpm vs torque curve instead of the calculated one', ...
    'Value', inp.torque_override, 'ValueChangedFcn', @(src, ~) onTorqueOverride(src.Value));
placeRow(cbTorqueOverride, nPt + 3);
gCurveBtns = uigridlayout(ptGrid, [1 4]);
gCurveBtns.Padding = [0 0 0 0];
gCurveBtns.ColumnSpacing = 4;
placeRow(gCurveBtns, nPt + 4);
uibutton(gCurveBtns, 'Text', 'Load file...', 'ButtonPushedFcn', @(~, ~) onLoadCurve());
uibutton(gCurveBtns, 'Text', 'Copy calculated', 'ButtonPushedFcn', @(~, ~) onCopyCalculatedCurve());
uibutton(gCurveBtns, 'Text', 'Add row', 'ButtonPushedFcn', @(~, ~) onAddCurveRow());
uibutton(gCurveBtns, 'Text', 'Delete row', 'ButtonPushedFcn', @(~, ~) onDeleteCurveRow());
curveTable = uitable(ptGrid, 'ColumnName', {'Motor speed (rpm)', 'Motor torque (Nm)'}, 'RowName', {}, ...
    'ColumnEditable', [true true], 'Data', inp.torque_override_curve, ...
    'CellEditCallback', @(~, ~) onCurveEdited(), 'CellSelectionCallback', @(~, evt) onCurveSelect(evt));
placeRow(curveTable, nPt + 5);
placeRow(uilabel(ptGrid, 'WordWrap', 'on', 'FontColor', [0.4 0.4 0.4], 'Text', ...
    ['Torque at the motor output shaft (motor efficiency already included). Load a .csv / .xlsx / .txt / .mat ' ...
    'with rpm in the first column and Nm in the second. The last rpm is the rev limit.']), nPt + 6);
curveSelRow = [];

%% Hardpoint tabs
tblF = makePointTab('Front Hardpoints', 'F');
tblR = makePointTab('Rear Hardpoints', 'R');
tblF.CellEditCallback = @(src, evt) onPoint(src, evt, 'F');
tblR.CellEditCallback = @(src, evt) onPoint(src, evt, 'R');

recalc();
if isfile(trackField.Value)
    runLap();
else
    lapLabel.Text = 'Track file not found - pick one with Browse';
end

%% ---------------- Callbacks ----------------

    function onScalar(f, value)
        inp = setInp(inp, f, value);
        recalc();
    end

    function onPoint(src, evt, suffix)
        r = evt.Indices(1);
        c = evt.Indices(2);
        f = [pointDefs{r, 1} suffix];
        coord = c - 1;
        if isCalculated(f, coord)
            src.Data{r, c} = evt.PreviousData;
            setStatus('That coordinate is calculated, not an input - edit the point it comes from.', 'warn');
            return
        end
        v = evt.NewData;
        if ischar(v) || isstring(v), v = str2double(v); end
        if ~isnumeric(v) || ~isscalar(v)
            src.Data{r, c} = evt.PreviousData;
            setStatus('Enter a number.', 'warn');
            return
        end
        inp.(f)(coord) = v;
        recalc();
    end

    function onReset()
        inp = vehicle_model_defaults();
        for k = 1:numel(scalarFields)
            h = scalarHandles{k};
            if isa(h, 'matlab.ui.control.DropDown')
                h.Value = char(getInp(inp, scalarFields{k}));
            else
                h.Value = getInp(inp, scalarFields{k});
            end
        end
        cbTorqueOverride.Value = inp.torque_override;
        curveTable.Data = inp.torque_override_curve;
        recalc();
    end

    function onTorqueOverride(on)
        if on && isempty(validCurveRows(curveTable.Data))
            curveTable.Data = calculatedCurve();   % start from the model's curve so there is something to edit
            inp.torque_override_curve = curveTable.Data;
        end
        inp.torque_override = on;
        recalc();
    end

    function onCurveEdited()
        inp.torque_override_curve = curveTable.Data;
        if inp.torque_override, recalc(); end
    end

    function onCurveSelect(evt)
        if isempty(evt.Indices), curveSelRow = []; else, curveSelRow = unique(evt.Indices(:, 1)); end
    end

    function onLoadCurve()
        [f, pth] = uigetfile({'*.csv;*.txt;*.xlsx;*.xls;*.mat', 'Torque curve (*.csv, *.txt, *.xlsx, *.mat)'}, ...
            'Choose an rpm vs torque curve');
        figure(fig);
        if isequal(f, 0), return, end
        try
            data = readCurveFile(fullfile(pth, f));
        catch err
            setStatus(['Could not read torque curve: ' err.message], 'error');
            return
        end
        curveTable.Data = data;
        inp.torque_override_curve = data;
        inp.torque_override = true;
        cbTorqueOverride.Value = true;
        recalc();
    end

    function onCopyCalculatedCurve()
        curveTable.Data = calculatedCurve();
        onCurveEdited();
    end

    function onAddCurveRow()
        d = curveTable.Data;
        if isempty(d), d = zeros(0, 2); end
        curveTable.Data = [d; NaN NaN];
        inp.torque_override_curve = curveTable.Data;   % NaN rows are ignored until filled in
    end

    function onDeleteCurveRow()
        d = curveTable.Data;
        if isempty(d), return, end
        rows = curveSelRow;
        rows = rows(rows >= 1 & rows <= size(d, 1));
        if isempty(rows), rows = size(d, 1); end
        d(rows, :) = [];
        curveTable.Data = d;
        curveSelRow = [];
        onCurveEdited();
    end

    function c = calculatedCurve()
        % The model's motor torque curve (override off) at its full 100 rpm resolution,
        % so copying it unchanged gives the same results as the calculated model
        tmp = inp;
        tmp.torque_override = false;
        v = vehicle_model_calc(tmp);
        keep = ~isnan(v.pt.motor_torque);
        c = round([v.pt.motor_rpm(keep), v.pt.motor_torque(keep)] * 100) / 100;
    end

    function placeRow(h, row)
        h.Layout.Row = row;
        h.Layout.Column = [1 3];
    end

    function onExport()
        assignin('base', 'inp', inp);
        assignin('base', 'veh', veh);
        assignin('base', 'lap', lapRes);
        assignin('base', 'lapLoads', lapLoads);
        assignin('base', 'lapChannels', lapCh);
        assignin('base', 'lapBaseline', baseRes);
        setStatus('Sent inp, veh, lap, lapLoads, lapChannels and lapBaseline to the base workspace.', 'ok');
    end

    function onToggleOutputs()
        if outTable.Visible
            outTable.Visible = 'off';
            right.RowHeight = {'1x', 24, 0};
            btnOutToggle.Text = [char(9656) ' Show results table'];
        else
            outTable.Visible = 'on';
            right.RowHeight = {'2x', 24, '1x'};
            btnOutToggle.Text = [char(9662) ' Hide results table'];
        end
    end

%% ---------------- Update ----------------

    function recalc()
        try
            [veh, calcCoords] = vehicle_model_calc(inp);
        catch err
            setStatus(['Calculation error: ' err.message], 'error');
            return
        end
        tblF.Data = pointData('F');
        tblR.Data = pointData('R');
        outTable.Data = outputData();
        G = veh.ggv;
        ggvTable.Data = round([G.v; G.gLat; G.gLong_acc; G.gLong_brk; G.gLong_powtrn; G.balance; G.brake_bias]' * 1e4) / 1e4;
        try
            drawAll();
            setStatus('Grey italic coordinates are calculated, not inputs.', 'ok');
        catch err
            setStatus(['Plot error: ' err.message], 'error');
        end
        if ~isempty(lapRes)
            if cbAutoLap.Value
                runLap();
            else
                lapLabel.Text = 'Inputs changed - press Run lap sim';
                lapLabel.FontColor = [0.85 0.45 0];
            end
        end
    end

%% ---------------- Lap sim ----------------

    function onBrowse()
        [f, pth] = uigetfile('*.mat', 'Choose a track / logged lap file', trackField.Value);
        figure(fig);
        if isequal(f, 0), return, end
        trackField.Value = fullfile(pth, f);
        runLap();
    end

    function runLap()
        file = trackField.Value;
        if ~isfile(file)
            lapLabel.Text = 'Track file not found';
            lapLabel.FontColor = [0.8 0 0];
            return
        end
        lapLabel.Text = 'Running lap sim...';
        lapLabel.FontColor = [0.3 0.3 0.3];
        drawnow;
        try
            lapRes = lap_sim(veh.ggv, file);
            lapInp = inp;
            lapLoads = lap_loads(veh, lapRes);
            lapCh = lap_channels(lapRes, lapLoads);
            lapShockLim = [veh.x_shock_compressed, veh.x_shock_extended];
        catch err
            lapRes = [];
            lapLoads = [];
            lapCh = [];
            lapLabel.Text = ['Lap sim error: ' err.message];
            lapLabel.FontColor = [0.8 0 0];
            return
        end
        lapLabel.FontColor = [0 0 0];
        if isempty(baseRes) || ~strcmp(baseFile, file)
            % First run (or a new track): this run becomes the baseline
            setBaseline();
        elseif ~isequaln(lapInp, baseInp)
            % A run with changed inputs: compare against the baseline
            ddCompare.Value = 'Baseline sim';
        end
        drawLap();
    end

    function setBaseline()
        baseRes = lapRes;
        baseInp = lapInp;
        baseFile = trackField.Value;
        baseCh = lapCh;
    end

    function onSetBaseline()
        if isempty(lapRes), return, end
        setBaseline();
        drawLap();
    end

    function onClearBaseline()
        baseRes = []; baseInp = []; baseFile = ''; baseCh = [];
        if strcmp(ddCompare.Value, 'Baseline sim'), ddCompare.Value = 'Logged lap'; end
        drawLap();
    end

    function drawLap()
        if isempty(lapRes), return, end
        r = lapRes;
        cSim = [0.85 0.33 0.10];
        cLog = [0.00 0.45 0.74];

        % What to compare against: the logged lap or the baseline sim (same track points)
        lg = [];
        refName = '';
        switch ddCompare.Value
            case 'Logged lap'
                lg = r.logged;
                refName = 'Logged';
            case 'Baseline sim'
                if ~isempty(baseRes) && strcmp(baseFile, trackField.Value) && numel(baseRes.s) == numel(r.s)
                    lg = struct('v', baseRes.vCar, 'gLat', baseRes.gLatSim, 'gLong', baseRes.gLongSim, ...
                        't', baseRes.tSim, 'lapTime', baseRes.lapTime);
                end
                refName = 'Baseline';
                cLog = [1 1 1];
        end
        overlay = ~isempty(lg);

        if overlay
            lapLabel.Text = sprintf('Sim %.2f s  |  %s %.2f s  (%+.2f s)', r.lapTime, refName, lg.lapTime, r.lapTime - lg.lapTime);
            if strcmp(refName, 'Baseline') && isequaln(lapInp, baseInp)
                lapLabel.Text = [lapLabel.Text '  - same inputs as baseline'];
            end
        else
            lapLabel.Text = sprintf('Sim lap time %.2f s', r.lapTime);
            if strcmp(ddCompare.Value, 'Baseline sim'), lapLabel.Text = [lapLabel.Text '  (no baseline for this track)']; end
        end
        if r.nGaps > 0
            lapLabel.Text = sprintf('%s  - %d points unsolved', lapLabel.Text, r.nGaps);
        end

        % Track map coloured by simulated speed
        cla(axTrack);
        scatter(axTrack, r.x, r.y, 10, r.vCar, 'filled');
        hold(axTrack, 'on');
        plot(axTrack, r.x(1), r.y(1), 'k>', 'MarkerFaceColor', 'k', 'MarkerSize', 8);
        hold(axTrack, 'off');
        axis(axTrack, 'equal');
        grid(axTrack, 'on');
        cb = colorbar(axTrack);
        cb.Label.String = 'vCar [m/s]';
        title(axTrack, 'Track (triangle = start)');
        xlabel(axTrack, 'X (m)'); ylabel(axTrack, 'Y (m)');

        trace(axLapV, r.vCar, lg, 'v', 'vCar [m/s]');
        hold(axLapV, 'on');
        %plot(axLapV, r.apex_s, r.vMaxAtApex, 'k.', 'MarkerSize', 6, 'DisplayName', 'Apex limit');
        hold(axLapV, 'off');
        trace(axLapLat, r.gLatSim, lg, 'gLat', 'gLat (g)');
        trace(axLapLong, r.gLongSim, lg, 'gLong', 'gLong (g)');

        cla(axLapDt);
        if overlay
            plot(axLapDt, r.s, r.tSim - lg.t, '-', 'Color', cSim, 'LineWidth', 1.2);
            if strcmp(refName, 'Baseline')
                ylabel(axLapDt, '\Deltat sim - baseline (s)');
            else
                ylabel(axLapDt, '\Deltat sim - log (s)');
            end
        else
            plot(axLapDt, r.s, r.radius, '-', 'Color', cSim, 'LineWidth', 1.2);
            ylabel(axLapDt, 'Radius (m)');
        end
        grid(axLapDt, 'on');
        xlabel(axLapDt, 'Distance (m)');
        linkaxes([axLapV axLapLat axLapLong axLapDt], 'x');
        xlim(axLapDt, [0 r.s(end)]);

        drawLoads(strcmp(refName, 'Baseline') && overlay);

        function trace(ax, ySim, lgd, fld, yl)
            cla(ax);
            plot(ax, r.s, ySim, '-', 'Color', cSim, 'LineWidth', 1.3, 'DisplayName', 'Simulated');
            if overlay
                hold(ax, 'on');
                plot(ax, r.s, lgd.(fld), '-', 'Color', cLog, 'LineWidth', 1.0, 'DisplayName', refName);
                hold(ax, 'off');
                legend(ax, 'Location', 'northeast');
            else
                legend(ax, 'off');
            end
            grid(ax, 'on');
            ylabel(ax, yl);
        end
    end

    function clearLoadAxes(a)
        % cla skips hidden handles, so also delete the shock limit ylines (HandleVisibility off)
        cla(a);
        delete(findall(a, 'Type', 'ConstantLine'));
        legend(a, 'off');
    end

    function drawLoads(showBase)
        if nargin > 0, loadsShowBase = showBase; end
        if isempty(lapCh)
            for a = axLd, clearLoadAxes(a); end
            loadsTable.Data = {};
            loadsNote.Text = '';
            return
        end
        names = lapCh.Properties.VariableNames;
        units = lapCh.Properties.VariableUnits;
        unitOf = @(n) units{strcmp(names, n)};

        % Keep the pickers in step with the channels this run has
        if ~ismember(ddX.Value, names), xVal = 'sLap'; else, xVal = ddX.Value; end
        ddX.Items = names;
        ddX.Value = xVal;
        lbCh.Items = names;
        for p = 1:4, panelSel{p} = panelSel{p}(ismember(panelSel{p}, names)); end
        lbCh.Value = panelSel{panelIdx()};

        x = lapCh.(xVal);
        asLine = ismember(xVal, {'sLap', 'tLap', 'Logged time'});   % monotonic: draw lines
        hasBase = loadsShowBase && ~isempty(baseCh) && height(baseCh) == height(lapCh);
        shown = [];
        for p = 1:4
            a = axLd(p);
            clearLoadAxes(a);
            xlabel(a, '');
            sel = panelSel{p};
            if isempty(sel)
                gLoadTraces.RowHeight{p} = 0;
                a.Visible = 'off';
                continue
            end
            gLoadTraces.RowHeight{p} = '1x';
            a.Visible = 'on';
            shown(end + 1) = p; %#ok<AGROW>
            cc = lines(max(numel(sel), 1));
            selUnits = cellfun(unitOf, sel, 'UniformOutput', false);
            mixed = numel(unique(selUnits)) > 1;
            hold(a, 'on');
            for j = 1:numel(sel)
                lbl = sel{j};
                if mixed, lbl = sprintf('%s (%s)', sel{j}, selUnits{j}); end
                y = lapCh.(sel{j});
                if asLine
                    plot(a, x, y, '-', 'Color', cc(j, :), 'LineWidth', 1, 'DisplayName', lbl);
                else
                    plot(a, x, y, '.', 'Color', cc(j, :), 'MarkerSize', 5, 'DisplayName', lbl);
                end
                if hasBase && ismember(sel{j}, baseCh.Properties.VariableNames) && ismember(xVal, baseCh.Properties.VariableNames)
                    if asLine
                        plot(a, baseCh.(xVal), baseCh.(sel{j}), '--', 'Color', cc(j, :), 'LineWidth', 0.8, ...
                            'DisplayName', [lbl ' (baseline)']);
                    else
                        plot(a, baseCh.(xVal), baseCh.(sel{j}), 'o', 'Color', cc(j, :), 'MarkerSize', 2, ...
                            'DisplayName', [lbl ' (baseline)']);
                    end
                end
            end
            if any(startsWith(sel, 'Shock length')) && ~isempty(lapShockLim)
                yline(a, lapShockLim(2), 'r--', 'Extended', 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
                yline(a, lapShockLim(1), 'r--', 'Compressed', 'HandleVisibility', 'off', 'LabelHorizontalAlignment', 'left');
            end
            hold(a, 'off');
            if mixed
                ylabel(a, '');
            elseif isscalar(sel)
                ylabel(a, sprintf('%s (%s)', sel{1}, selUnits{1}));
            else
                ylabel(a, sprintf('(%s)', selUnits{1}));
            end
            grid(a, 'on');
            axis(a, 'auto');
            lgd = legend(a, 'Location', 'northoutside', 'Orientation', 'horizontal');
            lgd.Box = 'off';
        end
        if ~isempty(shown)
            xlabel(axLd(shown(end)), sprintf('%s (%s)', xVal, unitOf(xVal)));
            if asLine
                linkaxes(axLd(shown), 'x');
                xlim(axLd(shown(end)), [min(x) max(x)]);
            else
                linkaxes(axLd, 'off');
            end
        end

        % Min / max of every channel
        fmt1 = @(v) arrayfun(@(y) sprintf('%.4g', y), v, 'UniformOutput', false);
        M = lapCh.Variables;
        loadsTable.Data = [names', fmt1(min(M, [], 1, 'omitnan'))', fmt1(max(M, [], 1, 'omitnan'))', units'];

        % Warnings
        msgs = {};
        if ~isempty(lapLoads)
            nLift = nnz(any(lapLoads.wheelLift, 1));
            nBot = nnz(any(lapLoads.shockBottomed, 1));
            nTop = nnz(any(lapLoads.shockTopped, 1));
            nGround = nnz(lapLoads.FRH <= 0 | lapLoads.RRH <= 0);
            if nLift, msgs{end + 1} = sprintf('Wheel lift at %d points.', nLift); end
            if nBot, msgs{end + 1} = sprintf('Shock past fully compressed at %d points.', nBot); end
            if nTop, msgs{end + 1} = sprintf('Shock past fully extended at %d points.', nTop); end
            if nGround, msgs{end + 1} = sprintf('Ride height at or below 0 at %d points.', nGround); end
        end
        if isempty(msgs)
            loadsNote.Text = 'No wheel lift, shock limits or ground contact around the lap.';
            loadsNote.FontColor = [0.3 0.3 0.3];
        else
            loadsNote.Text = strjoin(msgs, ' ');
            loadsNote.FontColor = [0.85 0.45 0];
        end
    end

    function p = panelIdx()
        p = find(strcmp(ddPanel.Items, ddPanel.Value), 1);
    end

    function onPanelPick()
        lbCh.Value = panelSel{panelIdx()};
    end

    function onChannelPick()
        panelSel{panelIdx()} = lbCh.Value;
        drawLoads();
    end

    function onExportChannels()
        if isempty(lapCh)
            setStatus('Run the lap sim first - there are no lap channels to export.', 'warn');
            return
        end
        [f, pth] = uiputfile({'*.csv', 'CSV (*.csv)'; '*.xlsx', 'Excel (*.xlsx)'; '*.mat', 'MATLAB table (*.mat)'}, ...
            'Export lap channels', 'lap_channels.csv');
        figure(fig);
        if isequal(f, 0), return, end
        exportChannels(fullfile(pth, f));
    end

    function exportChannels(file)
        T = lapCh;
        if cbExportPlotted.Value
            keep = unique([{ddX.Value}, panelSel{:}], 'stable');
            T = T(:, keep);
        end
        try
            [~, ~, ext] = fileparts(file);
            if strcmpi(ext, '.mat')
                lapChannels = T;
                save(file, 'lapChannels');
            else
                % Header = "channel [unit]"
                T.Properties.VariableNames = strcat(T.Properties.VariableNames, ' [', T.Properties.VariableUnits, ']');
                if isfile(file), delete(file); end   % writetable would otherwise only overwrite cells in an .xlsx
                writetable(T, file);
            end
            setStatus(sprintf('Exported %d channels x %d points to %s', width(T), height(T), file), 'ok');
        catch err
            setStatus(['Export failed: ' err.message], 'error');
        end
    end

    function setStatus(msg, level)
        status.Text = msg;
        switch level
            case 'error', status.FontColor = [0.8 0 0];
            case 'warn',  status.FontColor = [0.85 0.45 0];
            otherwise,    status.FontColor = [0.3 0.3 0.3];
        end
    end

%% ---------------- Tables ----------------

    function tbl = makePointTab(title, suffix)
        tab = uitab(inTabs, 'Title', title);
        g = uigridlayout(tab, [1 1]);
        tbl = uitable(g, 'ColumnName', {'Point', 'X long (mm)', 'Y lat (mm)', 'Z height (mm)'}, ...
            'RowName', {}, 'ColumnEditable', [false true true true], ...
            'ColumnFormat', {'char', 'numeric', 'numeric', 'numeric'}, ...
            'ColumnWidth', {150, 'auto', 'auto', 'auto'}, ...
            'Data', pointData(suffix));
        % Grey out calculated coordinates
        cells = zeros(0, 2);
        for r = 1:size(pointDefs, 1)
            for c = 1:3
                if isCalculated([pointDefs{r, 1} suffix], c)
                    cells(end + 1, :) = [r, c + 1]; %#ok<AGROW>
                end
            end
        end
        if ~isempty(cells)
            try
                addStyle(tbl, uistyle('BackgroundColor', [0.92 0.92 0.92], 'FontColor', [0.45 0.45 0.45], ...
                    'FontAngle', 'italic'), 'cell', cells);
            catch
                % uistyle needs R2019b+; edits to these cells are still blocked
            end
        end
    end

    function tf = isCalculated(f, coord)
        tf = isfield(calcCoords, f) && calcCoords.(f)(coord);
    end

    function data = pointData(suffix)
        n = size(pointDefs, 1);
        data = cell(n, 4);
        for r = 1:n
            p = veh.([pointDefs{r, 1} suffix]);
            data(r, :) = {pointDefs{r, 2}, p(1), p(2), p(3)};
        end
    end

    function data = outputData()
        v = veh;
        rows = {
            'Roll centre height',               v.h_roll_centreF,  v.h_roll_centreR,  'mm',   '%.2f';
            'Caster trail',                     v.x_caster_trailF, v.x_caster_trailR, 'mm',   '%.2f';
            'Scrub radius',                     v.x_scrub_radiusF, v.x_scrub_radiusR, 'mm',   '%.3f';
            'KPI angle',                        v.theta_KPIF,      v.theta_KPIR,      'deg',  '%.2f';
            'Caster angle',                     v.theta_casterF,   v.theta_casterR,   'deg',  '%.2f';
            'Virtual swing axle length',        v.x_VSAL_F,        v.x_VSAL_R,        'mm',   '%.1f';
            'Anti-dive (F) / anti-lift (R)',    v.r_anti_diveF,    v.r_anti_liftR,    '-',    '%.4f';
            'Anti-squat',                       NaN,               v.r_anti_squatR,   '-',    '%.4f';
            'Brake split',                      v.r_brake_splitF,  v.r_brake_splitR,  '-',    '%.2f';
            'Static loaded radius',             v.x_LoadedRadiusF, v.x_LoadedRadiusR, 'mm',   '%.2f';
            'Tyre spring rate',                 v.k_TyreF,         v.k_TyreR,         'N/mm', '%.2f';
            'Spring rate',                      v.k_springF,       v.k_springR,       'N/mm', '%.2f';
            'Motion ratio (selected)',          v.r_MRF,           v.r_MRR,           '-',    '%.2f';
            'Wheel rate',                       v.k_wheelF,        v.k_wheelR,        'N/mm', '%.2f';
            'Static shock length',              v.x_shock_staticF, v.x_shock_staticR, 'mm',   '%.2f';
            'ARB roll rate',                    v.k_ARB_rollF,     v.k_ARB_rollR,     'Nm/deg', '%.1f';
            'Total roll rate (incl. tyre)',     v.k_rollF,         v.k_rollR,         'Nm/deg', '%.1f';
            'Lateral load distribution (FLLD)', 100 * v.r_FLLD,    100 * (1 - v.r_FLLD), '%', '%.2f';
            'Heave rate (incl. tyre)',          v.k_heave_totalF,  v.k_heave_totalR,  'N/mm', '%.2f';
            'Gearing top speed',                v.pt.v_top,        NaN,               'm/s',  '%.1f';
            'Acceleration event time',          v.pt.t_accel,      NaN,               's',    '%.2f';
            'Skid pad speed',                   v.ggv.v_skidpad,   NaN,               'm/s',  '%.3f';
            'Axle mass',                        v.m_totalF,        v.m_totalR,        'kg',   '%.2f';
            'Sprung mass',                      v.m_sprungF,       v.m_sprungR,       'kg',   '%.2f';
            'Total mass',                       v.m_total,         NaN,               'kg',   '%.2f';
            'Total CoG height',                 v.h_total_cog,     NaN,               'mm',   '%.2f';
            'Total weight distribution (front)', 100 * v.r_mass_total, NaN,           '%',    '%.2f'};
        data = cell(size(rows, 1), 4);
        for r = 1:size(rows, 1)
            data(r, :) = {rows{r, 1}, fmt(rows{r, 2}, rows{r, 5}), fmt(rows{r, 3}, rows{r, 5}), rows{r, 4}};
        end
    end

%% ---------------- Plots ----------------

    function ax = makeAxes(parent, title)
        tab = uitab(parent, 'Title', title);
        g = uigridlayout(tab, [1 1]);
        ax = uiaxes(g);
    end

    function drawAll()
        draw3D();
        drawFrontView(axFF, 'F', 'Front axle - front view');
        drawFrontView(axFR, 'R', 'Rear axle - front view (looking forward)');
        drawSideView();
        drawGGV();
        drawPowertrain();
    end

    function drawGGV()
        % G-G curves at each speed, as on 'Tyres & Load Transfer Graphs' ("G-G Plot at Different Speeds")
        G = veh.ggv;
        ax = axGGV;
        cla(ax);
        hold(ax, 'on');
        cols = lines(numel(G.v));
        for k = 1:numel(G.v)
            lat = linspace(-G.gLat(k), G.gLat(k), 81);
            e = sqrt(max(0, 1 - (lat / G.gLat(k)).^2));
            up = min(G.gLong_powtrn(k), G.gLong_acc(k) * e);
            dn = -G.gLong_brk(k) * e;
            plot(ax, [lat, fliplr(lat), lat(1)], [up, fliplr(dn), up(1)], '-', 'Color', cols(k, :), 'LineWidth', 1.5, ...
                'DisplayName', sprintf('%.1f m/s (%s)', G.v(k), G.labels{k}));
        end
        hold(ax, 'off');
        axis(ax, 'equal');
        grid(ax, 'on');
        xlabel(ax, 'Lateral (g)'); ylabel(ax, 'Longitudinal (g)');
        legend(ax, 'Location', 'eastoutside');
        title(ax, 'G-G at different speeds');
    end

    function drawPowertrain()
        pt = veh.pt;
        ax = axPT;
        cla(ax);
        yyaxis(ax, 'left');
        plot(ax, pt.v_grid, pt.wheel_torque_v, '-', 'LineWidth', 1.5);
        ylabel(ax, 'Rear wheel torque (Nm)');
        yyaxis(ax, 'right');
        plot(ax, pt.v, pt.a / 9.81, '-', 'LineWidth', 1.5);
        ylabel(ax, 'Acceleration sim (g)');
        xlabel(ax, 'Speed (m/s)');
        xlim(ax, [0, max(pt.v_top * 1.1, 1)]);
        grid(ax, 'on');
        ttl = sprintf('Top speed %.1f m/s   |   %.0f m in %.2f s', pt.v_top, veh.x_accel_length, pt.t_accel);
        if isfield(pt, 'override') && pt.override, ttl = [ttl '   (user torque curve)']; end
        title(ax, ttl);
    end

    function draw3D()
        [az, el] = view(ax3);
        cla(ax3);
        hold(ax3, 'on');
        for s = 'FR'
            P = @(name) veh.([name s]);
            link3(P('uprfwd_outbrd'), P('upr_upright'), cWish, 2);
            link3(P('uprrwd_outbrd'), P('upr_upright'), cWish, 2);
            link3(P('lwrfwd_outbrd'), P('lwr_upright'), cWish, 2);
            link3(P('lwrrwd_outbrd'), P('lwr_upright'), cWish, 2);
            link3(P('upr_upright'), P('lwr_upright'), cUpr, 3);
            link3(P('toe_inbrd'), P('toe_outbrd'), cToe, 2);
            link3(P('prod_outbrd'), P('prod_inbrd'), cProd, 2);
            tri3([P('prod_inbrd'); P('rocker_centre'); P('rocker_spring')], cRocker);
            link3(P('rocker_spring'), P('spring_chassis'), cSpring, 4);
            link3(P('arb_upr_droplink'), P('arb_lwr_droplink'), cARB, 2);
            link3(P('arb_lwr_droplink'), P('arb_centre'), cARB, 2);

            pts = [P('upr_upright'); P('lwr_upright'); P('uprfwd_outbrd'); P('uprrwd_outbrd'); ...
                P('lwrfwd_outbrd'); P('lwrrwd_outbrd'); P('toe_outbrd'); P('toe_inbrd'); ...
                P('prod_outbrd'); P('prod_inbrd'); P('rocker_centre'); P('rocker_spring'); P('spring_chassis')];
            plot3(ax3, pts(:, 1), pts(:, 2), pts(:, 3), 'k.', 'MarkerSize', 10);
            plot3(ax3, pts(:, 1), -pts(:, 2), pts(:, 3), 'k.', 'MarkerSize', 10);

            [xA, yT, D, w, LR, hRC] = axleInfo(s);
            tyre3(xA, yT, LR, D / 2, w);
            tyre3(xA, -yT, LR, D / 2, w);
            plot3(ax3, xA, 0, hRC, 'p', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
        end
        xCoG = -veh.x_wheelbase * (1 - veh.r_mass_total);
        plot3(ax3, xCoG, 0, veh.h_total_cog, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'y');

        % Ground
        gx = [-veh.x_wheelbase - 400, 400];
        gy = max(veh.x_trackF, veh.x_trackR) / 2 + 300;
        patch(ax3, [gx(1) gx(2) gx(2) gx(1)], [-gy -gy gy gy], [0 0 0 0], [0.85 0.85 0.85], ...
            'FaceAlpha', 0.3, 'EdgeColor', 'none');

        hold(ax3, 'off');
        axis(ax3, 'equal');
        grid(ax3, 'on');
        view(ax3, az, el);
        xlabel(ax3, 'X long (mm)'); ylabel(ax3, 'Y lat (mm)'); zlabel(ax3, 'Z height (mm)');
        title(ax3, 'Suspension hardpoints  (red star = roll centre, yellow = CoG)');
    end

    function drawFrontView(ax, s, ttl)
        geo = veh.(['geo' s]);
        [~, yT, D, w, LR, hRC] = axleInfo(s);
        fv = geo.front_view_points;          % [upper upright; lower upright; upper chassis; lower chassis], [Y Z]
        ic = geo.ic_front_view;
        up3 = veh.(['upr_upright' s]);
        lo3 = veh.(['lwr_upright' s]);

        cla(ax);
        hold(ax, 'on');
        lim = yT + w / 2 + 150;
        plot(ax, [-lim lim], [0 0], 'k-', 'LineWidth', 1);
        for sg = [1 -1]
            Y = @(y) sg * y;
            % Tyre
            rectangle(ax, 'Position', [Y(yT) - w / 2, LR - D / 2, w, D], 'EdgeColor', cTyre, 'LineStyle', '-');
            % Wishbones and upright (projected onto the axle plane)
            plot(ax, Y([fv(3, 1) fv(1, 1)]), [fv(3, 2) fv(1, 2)], '-', 'Color', cWish, 'LineWidth', 2);
            plot(ax, Y([fv(4, 1) fv(2, 1)]), [fv(4, 2) fv(2, 2)], '-', 'Color', cWish, 'LineWidth', 2);
            plot(ax, Y([fv(1, 1) fv(2, 1)]), [fv(1, 2) fv(2, 2)], '-', 'Color', cUpr, 'LineWidth', 3);
            % Kingpin axis to the ground
            tG = -up3(3) / (lo3(3) - up3(3));
            yG = up3(2) + tG * (lo3(2) - up3(2));
            plot(ax, Y([up3(2) yG]), [up3(3) 0], ':', 'Color', cUpr);
            % Instant centre construction
            if all(isfinite(ic))
                plot(ax, Y([fv(1, 1) ic(1)]), [fv(1, 2) ic(2)], '--', 'Color', cWish);
                plot(ax, Y([fv(2, 1) ic(1)]), [fv(2, 2) ic(2)], '--', 'Color', cWish);
                plot(ax, Y([yT ic(1)]), [0 ic(2)], '--', 'Color', 'r');
                plot(ax, Y(ic(1)), ic(2), 'o', 'MarkerSize', 7, 'MarkerFaceColor', cIC, 'MarkerEdgeColor', 'k');
            end
            plot(ax, Y(yT), 0, 'k^', 'MarkerFaceColor', 'k');
        end
        if isfinite(hRC)
            plot(ax, 0, hRC, 'p', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
            text(ax, 15, hRC + 15, sprintf('RC %.1f mm', hRC), 'Color', 'r');
        end
        plot(ax, 0, veh.h_total_cog, 'ko', 'MarkerSize', 9, 'MarkerFaceColor', 'y');
        hold(ax, 'off');
        axis(ax, 'equal');
        xlim(ax, [-lim lim]);
        ylim(ax, [-60, max(D, veh.h_total_cog) + 120]);
        grid(ax, 'on');
        xlabel(ax, 'Y lat (mm)'); ylabel(ax, 'Z height (mm)');
        title(ax, sprintf('%s   (IC [%.0f, %.0f] mm, VSAL %.0f mm)', ttl, ic(1), ic(2), geo.x_VSAL));
    end

    function drawSideView()
        ax = axS;
        cla(ax);
        hold(ax, 'on');
        lims = [-veh.x_wheelbase - 450, 450];
        plot(ax, lims, [0 0], 'k-');
        notes = {};
        for s = 'FR'
            geo = veh.(['geo' s]);
            [xA, ~, D, ~, LR] = axleInfo(s);
            P = @(name) veh.([name s]);
            % Tyre outline
            th = linspace(0, 2 * pi, 60);
            plot(ax, xA + D / 2 * cos(th), LR + D / 2 * sin(th), '-', 'Color', cTyre);
            % Wishbone pickups and upright ball joints
            for pair = {{'uprfwd_outbrd', 'uprrwd_outbrd', 'upr_upright'}, {'lwrfwd_outbrd', 'lwrrwd_outbrd', 'lwr_upright'}}
                a = P(pair{1}{1}); b = P(pair{1}{2}); u = P(pair{1}{3});
                plot(ax, [a(1) b(1)], [a(3) b(3)], '-', 'Color', cWish, 'LineWidth', 2);
                plot(ax, [a(1) b(1) u(1)], [a(3) b(3) u(3)], 'k.', 'MarkerSize', 12);
            end
            up = P('upr_upright'); lo = P('lwr_upright');
            plot(ax, [up(1) lo(1)], [up(3) lo(3)], '-', 'Color', cUpr, 'LineWidth', 3);
            % Construction lines through the ball joints, z = s*(x - x_axle) + b
            L = geo.side_view_lines;
            ic = geo.ic_side_view;
            if all(isfinite(ic))
                xs = [0, ic(1)];
                for k = 1:2
                    plot(ax, xA + xs, L(k, 1) * xs + L(k, 2), '--', 'Color', cWish);
                end
                plot(ax, [xA, xA + ic(1)], [0, ic(2)], '--', 'Color', 'r');
                plot(ax, xA + ic(1), ic(2), 'o', 'MarkerSize', 7, 'MarkerFaceColor', cIC, 'MarkerEdgeColor', 'k');
            else
                xs = [-300, 300];
                for k = 1:2
                    plot(ax, xA + xs, L(k, 1) * xs + L(k, 2), '--', 'Color', cWish);
                end
                notes{end + 1} = sprintf('%s: side-view IC at infinity (parallel wishbones)', axleName(s)); %#ok<AGROW>
            end
            plot(ax, xA, 0, 'k^', 'MarkerFaceColor', 'k');
        end
        xCoG = -veh.x_wheelbase * (1 - veh.r_mass_total);
        plot(ax, xCoG, veh.h_total_cog, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
        hold(ax, 'off');
        axis(ax, 'equal');
        xlim(ax, lims);
        ylim(ax, [-60, 650]);
        grid(ax, 'on');
        xlabel(ax, 'X long (mm)'); ylabel(ax, 'Z height (mm)');
        ttl = sprintf('Side view   anti-dive F %.3f | anti-lift R %.3f | anti-squat R %.3f', ...
            veh.r_anti_diveF, veh.r_anti_liftR, veh.r_anti_squatR);
        if ~isempty(notes), ttl = {ttl, strjoin(notes, '; ')}; end
        title(ax, ttl);
    end

%% ---------------- Drawing helpers ----------------

    function link3(p, q, col, lw)
        if any(~isfinite([p q])), return, end
        plot3(ax3, [p(1) q(1)], [p(2) q(2)], [p(3) q(3)], '-', 'Color', col, 'LineWidth', lw);
        plot3(ax3, [p(1) q(1)], -[p(2) q(2)], [p(3) q(3)], '-', 'Color', col, 'LineWidth', lw);
    end

    function tri3(pts, col)
        if any(~isfinite(pts(:))), return, end
        patch(ax3, pts(:, 1), pts(:, 2), pts(:, 3), col, 'FaceAlpha', 0.6, 'EdgeColor', 'k');
        patch(ax3, pts(:, 1), -pts(:, 2), pts(:, 3), col, 'FaceAlpha', 0.6, 'EdgeColor', 'k');
    end

    function tyre3(xc, yc, zc, R, w)
        th = linspace(0, 2 * pi, 48);
        X = xc + R * cos(th);
        Z = zc + R * sin(th);
        surf(ax3, [X; X], [(yc - w / 2) * ones(size(th)); (yc + w / 2) * ones(size(th))], [Z; Z], ...
            'FaceColor', cTyre, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
        plot3(ax3, X, (yc - w / 2) * ones(size(th)), Z, '-', 'Color', cTyre);
        plot3(ax3, X, (yc + w / 2) * ones(size(th)), Z, '-', 'Color', cTyre);
    end

    function [xA, yT, D, w, LR, hRC] = axleInfo(s)
        if s == 'F'
            xA = 0;                 yT = veh.x_trackF / 2;  D = veh.x_Tyre_diameterF;
            w = veh.x_Tyre_widthF;  LR = veh.x_LoadedRadiusF; hRC = veh.h_roll_centreF;
        else
            xA = -veh.x_wheelbase;  yT = veh.x_trackR / 2;  D = veh.x_Tyre_diameterR;
            w = veh.x_Tyre_widthR;  LR = veh.x_LoadedRadiusR; hRC = veh.h_roll_centreR;
        end
    end

end

%% ---------------- Local functions ----------------

function s = fmt(v, f)
if isnan(v)
    s = '';
else
    s = sprintf(f, v);
end
end

function d = validCurveRows(d)
% Rows of an [rpm, torque] table with both values filled in
if isempty(d), d = zeros(0, 2); return, end
d = d(all(isfinite(d(:, 1:2)), 2), 1:2);
end

function d = readCurveFile(file)
% [rpm, torque] from a .mat (first numeric variable with 2+ columns) or a text / spreadsheet file
[~, ~, ext] = fileparts(file);
if strcmpi(ext, '.mat')
    s = load(file);
    names = fieldnames(s);
    d = [];
    for k = 1:numel(names)
        x = s.(names{k});
        if isnumeric(x) && ismatrix(x) && size(x, 2) >= 2 && size(x, 1) >= 2
            d = x;
            break
        end
    end
    if isempty(d), error('No numeric variable with rpm and torque columns found in the .mat file.'); end
else
    d = readmatrix(file);
end
if size(d, 2) < 2, error('Need rpm in column 1 and torque in column 2.'); end
d = validCurveRows(double(d));   % drops header rows and blanks
if size(d, 1) < 2, error('Found fewer than two rpm / torque rows.'); end
d = sortrows(d, 1);
end

function f = defaultTrackFile()
% Data\UGR26_FSUK_Ziru_Lap3.mat next to this GUI, in the current folder, or on the path
name = 'UGR26_FSUK_Ziru_Lap3.mat';
here = fileparts(mfilename('fullpath'));
cands = {fullfile(here, 'Data', name), fullfile(here, name), fullfile(pwd, 'Data', name), fullfile(pwd, name), which(name)};
f = fullfile('Data', name);
for k = 1:numel(cands)
    if ~isempty(cands{k}) && isfile(cands{k}), f = cands{k}; return, end
end
end

function v = getInp(inp, f)
% Value of an input field; 'name(i)' gives one element of a vector field
tok = regexp(f, '^(\w+)\((\d+)\)$', 'tokens', 'once');
if isempty(tok)
    v = inp.(f);
else
    v = inp.(tok{1})(str2double(tok{2}));
end
end

function inp = setInp(inp, f, v)
tok = regexp(f, '^(\w+)\((\d+)\)$', 'tokens', 'once');
if isempty(tok)
    inp.(f) = v;
else
    inp.(tok{1})(str2double(tok{2})) = v;
end
end

function n = axleName(s)
if s == 'F', n = 'Front'; else, n = 'Rear'; end
end
