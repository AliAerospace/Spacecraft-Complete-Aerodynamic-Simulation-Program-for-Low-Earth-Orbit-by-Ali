classdef LEOAEROSIMbyAliAerospace < matlab.apps.AppBase

    properties (Access = public)
        UIFigure            matlab.ui.Figure
        TabGroup            matlab.ui.container.TabGroup


        %tabs
        InputsTab           matlab.ui.container.Tab
        AtmosphereTab       matlab.ui.container.Tab
        ShadowsTab          matlab.ui.container.Tab
        PressureTab         matlab.ui.container.Tab
        PlottingTab      matlab.ui.container.Tab

        %inputs and config
        GeometryPanel       matlab.ui.container.Panel
        STLBaseLabel        matlab.ui.control.Label
        STLBaseEditField    matlab.ui.control.EditField
        LoadSTLButton       matlab.ui.control.Button
        CCWNoteLabel        matlab.ui.control.Label
        GeometryAxes        matlab.ui.control.UIAxes
        StatusTextAreaLabel matlab.ui.control.Label
        StatusTextArea      matlab.ui.control.TextArea

        %overview
        OverviewTab                 matlab.ui.container.Tab
        InputOverviewTextArea       matlab.ui.control.TextArea
        ComputedOverviewTextArea    matlab.ui.control.TextArea
        InputOverviewLabel          matlab.ui.control.Label
        ComputedOverviewLabel       matlab.ui.control.Label

        %velocity label for the flow
        AutoVelLabel                matlab.ui.control.Label

        %orientation 
        OrientationPanel    matlab.ui.container.Panel
        YawLabel            matlab.ui.control.Label
        YawEditField        matlab.ui.control.NumericEditField
        PitchLabel          matlab.ui.control.Label
        PitchEditField      matlab.ui.control.NumericEditField
        RollLabel           matlab.ui.control.Label
        RollEditField       matlab.ui.control.NumericEditField
        ApplyOrientationButton matlab.ui.control.Button

        %flow and some atmosphere
        FlowPanel           matlab.ui.container.Panel
        VelocityModeLabel   matlab.ui.control.Label
        VelocityModeDropDown matlab.ui.control.DropDown
        AltitudeLabel       matlab.ui.control.Label
        AltitudeEditField   matlab.ui.control.NumericEditField
        ManualVelLabel      matlab.ui.control.Label
        VxEditFieldLabel    matlab.ui.control.Label
        VxEditField         matlab.ui.control.NumericEditField


        AtmosphereModePanel matlab.ui.container.Panel
        AtmosphereModeLabel matlab.ui.control.Label
        AtmosphereModeDropDown matlab.ui.control.DropDown

        %physical parameters
        PhysicalPanel       matlab.ui.container.Panel
        TwLabel             matlab.ui.control.Label
        TwEditField         matlab.ui.control.NumericEditField
        AlphaNLabel         matlab.ui.control.Label
        AlphaNEditField     matlab.ui.control.NumericEditField
        AlphaTLabel         matlab.ui.control.Label
        AlphaTEditField     matlab.ui.control.NumericEditField

        RunButton           matlab.ui.control.Button

        %atmosphere 
        AtmosphereTable     matlab.ui.control.Table
        AtmTLabel           matlab.ui.control.Label
        AtmTEditField       matlab.ui.control.NumericEditField
        AtmTwLabel          matlab.ui.control.Label
        AtmTwEditField      matlab.ui.control.NumericEditField
        ApplyAtmButton      matlab.ui.control.Button
        LoadAtmButton       matlab.ui.control.Button
        SaveAtmButton       matlab.ui.control.Button

        %shadows
        ShadowAxes          matlab.ui.control.UIAxes
        RefreshShadowsButton matlab.ui.control.Button
        ShadowInfoLabel     matlab.ui.control.Label

        %pressures and forces
        PressureAxes        matlab.ui.control.UIAxes
        QuantityLabel       matlab.ui.control.Label
        QuantityDropDown    matlab.ui.control.DropDown
        ExportForcesButton  matlab.ui.control.Button
        ExportPressureButton matlab.ui.control.Button
        VisualisePressureButton matlab.ui.control.Button

        %coeff outputs
        CDLabel             matlab.ui.control.Label
        CDValueLabel        matlab.ui.control.Label
        CLLabel             matlab.ui.control.Label
        CLValueLabel        matlab.ui.control.Label
        CSLabel             matlab.ui.control.Label
        CSValueLabel        matlab.ui.control.Label
        ClLabel             matlab.ui.control.Label
        ClValueLabel        matlab.ui.control.Label
        CmLabel             matlab.ui.control.Label
        CmValueLabel        matlab.ui.control.Label
        CnLabel             matlab.ui.control.Label
        CnValueLabel        matlab.ui.control.Label
    end

    %non UI properties
    properties (Access = private)
        STLData
        RotatedVerts

        PanelProps
        BlockedPanels  logical
        BackwardPanels logical

        LastResults    struct

        ShadowFolder   char
    end

    %callbacks
    methods (Access = private)
        %decimel handling for the overview
        function out = fmt(app, x)

            if isempty(x) || isnan(x)
                out = sprintf('%12s', 'NaN');
                return;
            end

            if x == 0
                out = sprintf('%12s', '0');
                return;
            end

            if abs(x) >= 1e4 || abs(x) <= 1e-4
                s = sprintf('%.3e', x);
            else
                s = sprintf('%.6f', x);
            end
            out = sprintf('%12s', s);
        end

        %callback for atmosphere altering
        function AtmosphereModeChanged(app, ~)
            val = app.AtmosphereModeDropDown.Value;
            if contains(val, 'Manual override', 'IgnoreCase', true)
                %manual override adding soon
                app.TabGroup.SelectedTab = app.AtmosphereTab;
                app.StatusTextArea.Value = 'Switched to Atmosphere (Manual Override) tab.';
            end
        end


        %load stl
        function LoadSTLButtonPushed(app, ~)
            baseName = strtrim(app.STLBaseEditField.Value);
            if isempty(baseName)
                uialert(app.UIFigure, ...
                    'Please enter a base filename, e.g. "AURA14k".', ...
                    'Missing Filename');
                return;
            end
            fileName = [baseName '.stl'];

            try
                %attempt
                stlObj = stlread(fileName);
            catch ME
                msg = sprintf('Could not read file "%s".\n\nError:\n%s', ...
                    fileName, ME.message);
                uialert(app.UIFigure, msg, 'STL Load Error');
                app.StatusTextArea.Value = sprintf('Failed to load "%s".', fileName);
                return;
            end

            app.STLData = stlObj;
            app.RotatedVerts = stlObj.Points;

            %plot geometry in preview
            faces  = stlObj.ConnectivityList;
            verts  = stlObj.Points;

            cla(app.GeometryAxes);
            trisurf(faces, verts(:,1), verts(:,2), verts(:,3), ...
                'Parent', app.GeometryAxes, ...
                'EdgeColor', 'k', 'FaceColor', [0.8 0.8 1.0]);
            axis(app.GeometryAxes, 'equal');
            grid(app.GeometryAxes, 'on');
            view(app.GeometryAxes, 3);
            xlabel(app.GeometryAxes, 'X [m]');
            ylabel(app.GeometryAxes, 'Y [m]');
            zlabel(app.GeometryAxes, 'Z [m]');

            app.StatusTextArea.Value = sprintf('Loaded "%s" successfully.', fileName);
        end

        function QuantityChanged(app, ~)
            plotPressure(app);
        end

        %callback apply orientation button
        function ApplyOrientationButtonPushed(app, ~)
            if isempty(app.STLData)
                uialert(app.UIFigure, ...
                    'Load an STL first before applying an orientation.', ...
                    'No Geometry');
                return;
            end

            yawDeg   = app.YawEditField.Value;
            pitchDeg = app.PitchEditField.Value;
            rollDeg  = app.RollEditField.Value;

            %right-hand-rule: yaw, pitch, and roll are positive clockwise when viewed along
            %each ordinates positive direction
            psi   = deg2rad(-yawDeg);
            theta = deg2rad(pitchDeg);
            phi   = deg2rad(-rollDeg);

            %rotation matrices
            Rpsi = [cos(psi), -sin(psi), 0;
                sin(psi), cos(psi),  0;
                0,                 0,                  1];
            Rtheta = [cos(theta), 0, sin(theta);
                0,                   1, 0;
                -sin(theta), 0, cos(theta)];
            Rphi = [1, 0,                0;
                0, cos(phi), -sin(phi);
                0, sin(phi),  cos(phi)];

            R_ypr = Rpsi * Rtheta * Rphi;

            verts = app.STLData.Points;
            rotVerts = (R_ypr * verts.').';
            app.RotatedVerts = rotVerts;

            faces = app.STLData.ConnectivityList;

            cla(app.GeometryAxes);
            trisurf(faces, rotVerts(:,1), rotVerts(:,2), rotVerts(:,3), ...
                'Parent', app.GeometryAxes, ...
                'EdgeColor', 'k', 'FaceColor', [0.8 0.8 1.0]);
            axis(app.GeometryAxes, 'equal');
            grid(app.GeometryAxes, 'on');
            view(app.GeometryAxes, 3);
            xlabel(app.GeometryAxes, 'X [m]');
            ylabel(app.GeometryAxes, 'Y [m]');
            zlabel(app.GeometryAxes, 'Z [m]');

            app.StatusTextArea.Value = sprintf('Applied yaw=%.1f°, pitch=%.1f°, roll=%.1f°.', ...
                yawDeg, pitchDeg, rollDeg);
        end

        function RunButtonPushed(app, ~)
            try
                results = runAerodynamics(app);
            catch ME
                uialert(app.UIFigure, ME.message, 'Run error');
                return;
            end

            app.CDValueLabel.Text = sprintf('%.4f', results.CD);
            app.CLValueLabel.Text = sprintf('%.4f', results.CL);
            app.CSValueLabel.Text = sprintf('%.4f', results.CS);
            app.ClValueLabel.Text = sprintf('%.4f', results.Cl);
            app.CmValueLabel.Text = sprintf('%.4f', results.Cm);
            app.CnValueLabel.Text = sprintf('%.4f', results.Cn);

            app.StatusTextArea.Value = { ...
                'Ray-tracing completed (shadows and caching).'; ...
                sprintf('Panels: %d, blocked: %d', ...
                results.NPanels, results.NBlocked); ...
                sprintf('Shadow file: %s', results.shadowFilePath) };

            plotShadows(app);
            app.TabGroup.SelectedTab = app.ShadowsTab;
            %keep in sync
            updateInputOverview(app);
            updateComputedOverview(app);
        end

        
        function RefreshShadowsButtonPushed(app, ~)
            if isempty(app.STLData)
                uialert(app.UIFigure, ...
                    'Load an STL first before computing shadows.', ...
                    'No Geometry');
                return;
            end

            try
                results = runAerodynamics(app);
            catch ME
                uialert(app.UIFigure, ME.message, 'Shadow error');
                return;
            end

            app.StatusTextArea.Value = { ...
                'Ray-tracing completed (shadows and caching) from Refresh button.'; ...
                sprintf('Panels: %d, blocked: %d', ...
                results.NPanels, results.NBlocked, results.NBackward); ...
                sprintf('Shadow file: %s', results.shadowFilePath) };
            plotShadows(app);

            app.TabGroup.SelectedTab = app.ShadowsTab;
        end

        function VisualisePressureButtonPushed(app, ~)

            if isempty(app.LastResults)
                app.StatusTextArea.Value = 'No simulation results available.';
                return;
            end

            plotPressure(app);
            app.StatusTextArea.Value = 'Pressure visualisation updated.';
        end

        %auto update orbital velocity
        function updateAutoVelocity(app)
            Gr = 6.6743e-11;        % gravitational constant
            Me = 5.972e24;          % Earth mass [kg]
            Re = 6371000;           % Earth radius [m]
            h  = app.AltitudeEditField.Value * 1000;  % to m
            if h < 0
                h = 0;
            end
            V = sqrt(Gr * Me / (Re + h));
            app.AutoVelLabel.Text = sprintf( ...
                'Orbital speed (auto) [m/s]: %.2f  (calculated using momentum balance)', V);
        end

        function AltitudeChanged(app, ~)
            updateAutoVelocity(app);
        end

        function VelocityModeChanged(app, ~)
            val = app.VelocityModeDropDown.Value;
            isAuto = contains(val, 'Automatic', 'IgnoreCase', true);

            vis = ~isAuto;  

            app.ManualVelLabel.Visible   = vis;
            app.VxEditFieldLabel.Visible = vis;
            app.VxEditField.Visible      = vis;
            updateAutoVelocity(app);
        end


    end

    %components
    methods (Access = private)

        function createComponents(app)
            %main fig
            app.UIFigure = uifigure('Name', 'Spacecraft LEO Rarefied Aerodynamics Complete Simulator by AliAerospace');
            app.UIFigure.Position = [100 100 1200 700];

            %tab group
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [1 1 1200 700];

            %inputs and config tab
            app.InputsTab = uitab(app.TabGroup, 'Title', 'Inputs & Configuration');

            %geometry panel
            app.GeometryPanel = uipanel(app.InputsTab, ...
                'Title', 'Geometry', ...
                'Position', [20 300 560 360]);

            app.STLBaseLabel = uilabel(app.GeometryPanel, ...
                'Text', 'STL base filename (no extension):', ...
                'Position', [10 320 250 20]);

            app.STLBaseEditField = uieditfield(app.GeometryPanel, 'text', ...
                'Position', [260 320 150 22], ...
                'Value', 'AURA14k');

            app.LoadSTLButton = uibutton(app.GeometryPanel, 'push', ...
                'Text', 'Load Geometry', ...
                'Position', [420 320 120 22], ...
                'ButtonPushedFcn', @(src,evt)LoadSTLButtonPushed(app, evt));

            app.CCWNoteLabel = uilabel(app.GeometryPanel, ...
                'Text', 'Note: STL facets must be indexed in counter-clockwise order, most programs account for this automatically (such as Fusion 360).', ...
                'Position', [10 295 530 20], ...
                'FontAngle', 'italic');

            app.GeometryAxes = uiaxes(app.GeometryPanel, ...
                'Position', [10 10 540 280]);
            title(app.GeometryAxes, 'Geometry Preview');
            xlabel(app.GeometryAxes, 'X [m]');
            ylabel(app.GeometryAxes, 'Y [m]');
            zlabel(app.GeometryAxes, 'Z [m]');

            %status text area
            app.StatusTextAreaLabel = uilabel(app.InputsTab, ...
                'Text', 'Status:', ...
                'Position', [20 270 40 20]);

            app.StatusTextArea = uitextarea(app.InputsTab, ...
                'Position', [70 210 510 80], ...
                'Editable', 'off', ...
                'Value', {'Ready.'});

            %orientation panel
            app.OrientationPanel = uipanel(app.InputsTab, ...
                'Title', 'Attitude (Yaw-Pitch-Roll, degrees)', ...
                'Position', [600 500 580 160]);

            app.YawLabel = uilabel(app.OrientationPanel, ...
                'Text', 'Yaw (ψ):', ...
                'Position', [10 100 60 20]);
            app.YawEditField = uieditfield(app.OrientationPanel, 'numeric', ...
                'Position', [80 100 80 22], ...
                'Value', 0);

            app.PitchLabel = uilabel(app.OrientationPanel, ...
                'Text', 'Pitch (θ):', ...
                'Position', [180 100 70 20]);
            app.PitchEditField = uieditfield(app.OrientationPanel, 'numeric', ...
                'Position', [260 100 80 22], ...
                'Value', 0);

            app.RollLabel = uilabel(app.OrientationPanel, ...
                'Text', 'Roll (φ):', ...
                'Position', [360 100 60 20]);
            app.RollEditField = uieditfield(app.OrientationPanel, 'numeric', ...
                'Position', [430 100 80 22], ...
                'Value', 0);

            app.ApplyOrientationButton = uibutton(app.OrientationPanel, 'push', ...
                'Text', 'Apply Orientation', ...
                'Position', [220 50 150 26], ...
                'ButtonPushedFcn', @(src,evt)ApplyOrientationButtonPushed(app, evt));

            %flow panel
            app.FlowPanel = uipanel(app.InputsTab, ...
                'Title', 'Flow Conditions', ...
                'Position', [600 300 580 190]);

            app.VelocityModeLabel = uilabel(app.FlowPanel, ...
                'Text', 'Velocity Mode:', ...
                'Position', [10 130 100 20]);

            app.VelocityModeDropDown = uidropdown(app.FlowPanel, ...
                'Items', {'Automatic (orbital from altitude)', ...
                'Manual bulk speed'}, ...
                'Position', [120 130 250 22], ...
                'Value', 'Automatic (orbital from altitude)', ...
                'ValueChangedFcn', @(src,evt)VelocityModeChanged(app, evt));

            app.AltitudeLabel = uilabel(app.FlowPanel, ...
                'Text', 'Altitude h [km] (for auto mode):', ...
                'Position', [10 100 190 20]);

            app.AltitudeEditField = uieditfield(app.FlowPanel, 'numeric', ...
                'Position', [205 100 80 22], ...
                'Value', 500, ...
                'ValueChangedFcn', @(src,evt)AltitudeChanged(app, evt));

            %auto orbital velocity shown always
            app.AutoVelLabel = uilabel(app.FlowPanel, ...
                'Text', 'Orbital speed (auto) [m/s]: (calculated using momentum balance)', ...
                'Position', [10 75 540 20]);

            %hidden V field until required
            app.ManualVelLabel = uilabel(app.FlowPanel, ...
                'Text', 'Manual V components [m/s]:', ...
                'Position', [10 45 180 20], ...
                'Visible', 'off');

            app.VxEditFieldLabel = uilabel(app.FlowPanel, ...
                'Text', 'Vx:', ...
                'Position', [200 45 25 20], ...
                'Visible', 'off');
            app.VxEditField = uieditfield(app.FlowPanel, 'numeric', ...
                'Position', [225 45 80 22], ...
                'Value', 7500, ...
                'Visible', 'off');


            %atmosphere mode panel
            app.AtmosphereModePanel = uipanel(app.InputsTab, ...
                'Title', 'Atmosphere Mode', ...
                'Position', [600 210 280 80]);
            app.AtmosphereModePanel.Visible = 'off';

            app.AtmosphereModeLabel = uilabel(app.AtmosphereModePanel, ...
                'Text', 'Atmosphere Source:', ...
                'Position', [10 30 130 20]);

            app.AtmosphereModeDropDown = uidropdown(app.AtmosphereModePanel, ...
                'Items', {'Automatic (NRLMSISE-00)', 'Manual override (see Atmosphere tab)'}, ...
                'Position', [140 30 130 22], ...
                'Value', 'Automatic (NRLMSISE-00)', ...
                'ValueChangedFcn', @(src,evt)AtmosphereModeChanged(app, evt));

            %physical parameters panel
            app.PhysicalPanel = uipanel(app.InputsTab, ...
                'Title', 'Physical Parameters', ...
                'Position', [600 190 340 120]);
            %Tw
            app.TwLabel = uilabel(app.PhysicalPanel, ...
                'Text', 'Wall temperature $T_w$ [K]:', ...
                'Position', [10 70 170 20], 'Interpreter', 'latex');
            app.TwEditField = uieditfield(app.PhysicalPanel, 'numeric', ...
                'Position', [190 70 80 22], ...
                'Value', 300);

            %alpha_n
            app.AlphaNLabel = uilabel(app.PhysicalPanel, ...
                'Position', [10 45 260 20], ...
                'Interpreter', 'latex', ...
                'Text', '$\alpha_n$ (normal accommodation coefficient):');
            app.AlphaNEditField = uieditfield(app.PhysicalPanel, 'numeric', ...
                'Position', [260 45 40 22], ...
                'Value', 1);

            %alpha_t
            app.AlphaTLabel = uilabel(app.PhysicalPanel, ...
                'Position', [10 20 260 20], ...
                'Interpreter', 'latex', ...
                'Text', '$\alpha_t$ (tangential accommodation coefficient):');
            app.AlphaTEditField = uieditfield(app.PhysicalPanel, 'numeric', ...
                'Position', [260 20 40 22], ...
                'Value', 1);

            %run button
            app.RunButton = uibutton(app.InputsTab, 'push', ...
                'Text', 'Begin Simulation', ...
                'Position', [600 170 580 30], ...
                'ButtonPushedFcn', @(src,evt)RunButtonPushed(app, evt));

            %atmosphere table
            app.AtmosphereTab = uitab(app.TabGroup, 'Title', 'Atmosphere (Manual Override)');
            app.AtmosphereTab.Parent = []; %hidden for now, will be added in update
            app.AtmosphereTable = uitable(app.AtmosphereTab, ...
                'Position', [20 100 600 520], ...
                'ColumnName', {'Species', 'Mass density ρ [kg/m^3]', 'Number density n [1/m^3]'}, ...
                'ColumnEditable', [false true true]);

            speciesNames = {'He'; 'O'; 'N2'; 'O2'; 'Ar'; 'H'; 'N'; 'AnomO'};
            app.AtmosphereTable.Data = [speciesNames, num2cell(zeros(8,1)), num2cell(zeros(8,1))];

            app.AtmTLabel = uilabel(app.AtmosphereTab, ...
                'Text', 'Gas temperature T [K]:', ...
                'Position', [650 570 150 20]);
            app.AtmTEditField = uieditfield(app.AtmosphereTab, 'numeric', ...
                'Position', [800 570 80 22], ...
                'Value', 1100);

            app.AtmTwLabel = uilabel(app.AtmosphereTab, ...
                'Text', 'Wall temperature Tw [K]:', ...
                'Position', [650 540 150 20]);
            app.AtmTwEditField = uieditfield(app.AtmosphereTab, 'numeric', ...
                'Position', [800 540 80 22], ...
                'Value', 300);


            %shadows tab
            app.ShadowsTab = uitab(app.TabGroup, 'Title', 'Ray-Tracing (Shadowed Surface Visual)');

            app.ShadowAxes = uiaxes(app.ShadowsTab, ...
                'Position', [10 60 1180 630]);
            title(app.ShadowAxes, 'Ray-Traced Shadowed Geometry');
            xlabel(app.ShadowAxes, 'X');
            ylabel(app.ShadowAxes, 'Y');
            zlabel(app.ShadowAxes, 'Z');

            app.RefreshShadowsButton = uibutton(app.ShadowsTab, 'push', ...
                'Text', 'Refresh Shadows', ...
                'Position', [500 10 200 30], ...
                'ButtonPushedFcn', @(src,evt)RefreshShadowsButtonPushed(app, evt));

            %pressure and forces tab
            app.PressureTab = uitab(app.TabGroup, 'Title', 'Pressures & Shear Visual');

            app.PressureAxes = uiaxes(app.PressureTab, ...
                'Position', [20 100 700 520]);
            title(app.PressureAxes, 'Pressure / Stress Distribution');
            xlabel(app.PressureAxes, 'X');
            ylabel(app.PressureAxes, 'Y');
            zlabel(app.PressureAxes, 'Z');

            app.QuantityLabel = uilabel(app.PressureTab, ...
                'Text', 'Quantity:', ...
                'Position', [750 580 60 20]);
            app.QuantityDropDown = uidropdown(app.PressureTab, ...
                'Items', {'Incident pressure [N]', 'Reflected pressure [N]', 'Shear stress [N]'}, ...
                'Value', 'Incident pressure [N]', ...
                'Position', [820 580 150 22]);


            app.VisualisePressureButton = uibutton(app.PressureTab, 'push', ...
                'Text', 'Visualise Pressure', ...
                'Position', [750 540 220 26], ...
                'ButtonPushedFcn', @(src,evt)VisualisePressureButtonPushed(app, evt));

            app.QuantityDropDown.ValueChangedFcn = @(src,evt)QuantityChanged(app, evt);


            %overview tab
            app.OverviewTab = uitab(app.TabGroup, 'Title', 'Inputs & Outputs Overview');

            app.InputOverviewLabel = uilabel(app.OverviewTab, ...
                'Text', 'Input Variables Overview', ...
                'FontWeight', 'bold', ...
                'Position', [20 640 250 20]);

            app.InputOverviewTextArea = uitextarea(app.OverviewTab, ...
                'Position', [20 60 550 570], ...
                'Editable', 'off');

            app.ComputedOverviewLabel = uilabel(app.OverviewTab, ...
                'Text', 'Computed Variables Overview', ...
                'FontWeight', 'bold', ...
                'Position', [600 640 260 20]);


            app.ComputedOverviewTextArea = uitextarea(app.OverviewTab, ...
                'Position', [600 60 550 570], ...
                'Editable', 'off');

            %plotting tab coming soon
            app.PlottingTab = uitab(app.TabGroup, 'Title', 'Plotting');

            uilabel(app.PlottingTab, ...
                'Text', sprintf('UPDATE COMING SOON...\n(my raw code already does this, this UI is for showcasing on GitHub)'), ...
                'FontSize', 22, ...
                'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', ...
                'Position', [0 300 1200 80]);

            %display force moment coeff
            y0 = 450; dy = 30;
            app.CDLabel = uilabel(app.PressureTab, ...
                'Text', 'CD:', ...
                'Position', [750 y0 40 20]);
            app.CDValueLabel = uilabel(app.PressureTab, ...
                'Text', 'N/A', ...
                'Position', [790 y0 150 20]);

            app.CLLabel = uilabel(app.PressureTab, ...
                'Text', 'CL:', ...
                'Position', [750 y0-dy 40 20]);
            app.CLValueLabel = uilabel(app.PressureTab, ...
                'Text', 'N/A', ...
                'Position', [790 y0-dy 150 20]);

            app.CSLabel = uilabel(app.PressureTab, ...
                'Text', 'CS:', ...
                'Position', [750 y0-2*dy 40 20]);
            app.CSValueLabel = uilabel(app.PressureTab, ...
                'Text', 'N/A', ...
                'Position', [790 y0-2*dy 150 20]);

            app.ClLabel = uilabel(app.PressureTab, ...
                'Text', 'Cl:', ...
                'Position', [750 y0-3*dy 40 20]);
            app.ClValueLabel = uilabel(app.PressureTab, ...
                'Text', 'N/A', ...
                'Position', [790 y0-3*dy 150 20]);

            app.CmLabel = uilabel(app.PressureTab, ...
                'Text', 'Cm:', ...
                'Position', [750 y0-4*dy 40 20]);
            app.CmValueLabel = uilabel(app.PressureTab, ...
                'Text', 'N/A', ...
                'Position', [790 y0-4*dy 150 20]);

            app.CnLabel = uilabel(app.PressureTab, ...
                'Text', 'Cn:', ...
                'Position', [750 y0-5*dy 40 20]);
            app.CnValueLabel = uilabel(app.PressureTab, ...
                'Text', 'N/A', ...
                'Position', [790 y0-5*dy 150 20]);

            updateAutoVelocity(app);   % initialise auto-speed label
        end

        %input variables overview text
        function updateInputOverview(app)
            lines = {};


            lines{end+1} = sprintf('STL base filename: %s', app.STLBaseEditField.Value);
            lines{end+1} = sprintf('Yaw [deg]:   %.3f', app.YawEditField.Value);
            lines{end+1} = sprintf('Pitch [deg]: %.3f', app.PitchEditField.Value);
            lines{end+1} = sprintf('Roll [deg]:  %.3f', app.RollEditField.Value);

            lines{end+1} = ' ';
            lines{end+1} = sprintf('Velocity mode: %s', app.VelocityModeDropDown.Value);
            lines{end+1} = sprintf('Altitude h [km]: %.3f', app.AltitudeEditField.Value);
            lines{end+1} = sprintf('Atmosphere source: %s', app.AtmosphereModeDropDown.Value);

            if contains(app.VelocityModeDropDown.Value, 'Manual', 'IgnoreCase', true)
                lines{end+1} = sprintf('Manual bulk speed |U| [m/s]: %.3f', app.VxEditField.Value);
            else
                lines{end+1} = '(Velocity magnitude computed from orbital speed)';
            end

            lines{end+1} = ' ';
            lines{end+1} = sprintf('Wall temperature T_w [K]: %.3f', app.TwEditField.Value);
            lines{end+1} = sprintf('Normal accom. coeff. alpha_n: %.3f', app.AlphaNEditField.Value);
            lines{end+1} = sprintf('Tangential accom. coeff. alpha_t: %.3f', app.AlphaTEditField.Value);

            app.InputOverviewTextArea.Value = lines;
        end

        function updateComputedOverview(app)

            lines = {};

            if isempty(app.LastResults)
                lines{end+1} = 'No aerodynamic computation performed yet.';
                app.ComputedOverviewTextArea.Value = lines;
                return;
            end

            R = app.LastResults;

            lines{end+1} = sprintf('CD (drag coefficient):              %s', fmt(app, R.CD));
            lines{end+1} = sprintf('CL (lift coefficient):              %s', fmt(app, R.CL));
            lines{end+1} = sprintf('CS (side-force coefficient):        %s', fmt(app, R.CS));
            lines{end+1} = sprintf('Cl (roll moment coeff):             %s', fmt(app, R.Cl));
            lines{end+1} = sprintf('Cm (pitch moment coeff):            %s', fmt(app, R.Cm));
            lines{end+1} = sprintf('Cn (yaw moment coeff):              %s', fmt(app, R.Cn));
            lines{end+1} = ' ';

            lines{end+1} = sprintf('Centre of Gravity CG [m]:           [%s, %s, %s]', ...
                fmt(app, R.CG(1)), fmt(app, R.CG(2)), fmt(app, R.CG(3)));

            lines{end+1} = sprintf('Unrotated reference length [m]:     %s', fmt(app, R.lunrefx));
            lines{end+1} = sprintf('Unrotated contributing area [m^2]:  %s', fmt(app, R.Aunref));
            lines{end+1} = ' ';

            lines{end+1} = sprintf('Drag force D [N]:                   %s', fmt(app, R.FD));
            lines{end+1} = sprintf('Lift force L [N]:                   %s', fmt(app, R.FL));
            lines{end+1} = sprintf('Side force S [N]:                   %s', fmt(app, R.FS));
            lines{end+1} = ' ';

            lines{end+1} = sprintf('Velocity magnitude |U| [m/s]:       %s', fmt(app, R.Umag));
            lines{end+1} = sprintf('Speed ratio S:                      %s', fmt(app, R.Sratio));
            lines{end+1} = sprintf('Freestream temperature T [K]:       %s', fmt(app, R.T));
            lines{end+1} = sprintf('Freestream density rho [kg/m^3]:    %s', fmt(app, R.rho));
            lines{end+1} = sprintf('Average molecular mass m [kg]:      %s', fmt(app, R.m));
            lines{end+1} = ' ';

            lines{end+1} = sprintf('CAb (body-axis axial coeff):        %s', fmt(app, R.CAb));
            lines{end+1} = sprintf('CSb (body-axis side coeff):         %s', fmt(app, R.CSb));
            lines{end+1} = sprintf('CNb (body-axis normal coeff):       %s', fmt(app, R.CNb));

            app.ComputedOverviewTextArea.Value = lines;

        end
        %shadow and cache
        function results = runAerodynamics(app)

            if isempty(app.STLData)
                error('No geometry loaded. Please load an STL file first.');
            end

            stlobjectdata     = app.STLData;
            facets            = stlobjectdata.ConnectivityList;
            unrotatedvertices = stlobjectdata.Points;
            N                 = size(facets, 1);

            an = app.AlphaNEditField.Value;
            at = app.AlphaTEditField.Value;
            Tw = app.TwEditField.Value;

            %STL name match
            baseName = strtrim(app.STLBaseEditField.Value);
            if isempty(baseName)
                baseName = 'AURA14k';
            end
            stlName = baseName;



            %attitude and rotation
            yawDeg   = app.YawEditField.Value;
            pitchDeg = app.PitchEditField.Value;
            rollDeg  = app.RollEditField.Value;

            %right-hand-rule, matching your original code exactly
            psi   = deg2rad(-yawDeg);     %MATLAB axes correction
            theta = deg2rad(pitchDeg);
            phi   = deg2rad(-rollDeg);    %MATLAB axes correction

            Rpsi = [cos(psi), -sin(psi), 0;
                sin(psi),  cos(psi), 0;
                0,        0,   1];

            Rtheta = [cos(theta), 0, sin(theta);
                0,   1,      0;
                -sin(theta), 0, cos(theta)];

            Rphi = [1,        0,         0;
                0,  cos(phi), -sin(phi);
                0,  sin(phi),  cos(phi)];

            %yaw-pitch-roll rotation sequence
            R_ypr = Rpsi * Rtheta * Rphi;

            vertices = (R_ypr * unrotatedvertices.').';
            app.RotatedVerts = vertices;      % store for later plotting

            psi_deg   = rad2deg(-psi);
            theta_deg = rad2deg(theta);
            phi_deg   = rad2deg(-phi);

            psi_str   = sprintf('ps%d', round(psi_deg));
            theta_str = sprintf('th%d', round(theta_deg));
            phi_str   = sprintf('ph%d', round(phi_deg));

            shadowFileName = sprintf('%s_%s_%s_%s.mat', stlName, psi_str, theta_str, phi_str);
            shadowFilePath = fullfile(app.ShadowFolder, shadowFileName);


            %atmosphere
            alt_km = app.AltitudeEditField.Value;

            %auto NRLMSIS
            if contains(app.AtmosphereModeDropDown.Value, 'Automatic', 'IgnoreCase', true)

                %%%%%%%% USER. YOU CAN MANUALLY ALTER THESE IF YOU WISH %%%%%%%%
                year = 2024;
                dayOfYear = 1;
                UTseconds = 0;
                longitude = 0;
                latitude = 0;
                localApparentSolarTime = 0;
                %%%%%%%% USER. YOU CAN MANUALLY ALTER THESE IF YOU WISH %%%%%%%%

                persistent SWdata
                if isempty(SWdata)
                    SWdata = aeroReadSpaceWeatherData('SWLast5Years.csv');
                end

                [f107avg, f107daily, magIdx] = fluxSolarAndGeomagnetic( ...
                    year, dayOfYear, UTseconds, SWdata);

                %NRLMSISE-00 call
                [Tvec, rhovec] = atmosnrlmsise00( alt_km*1000, ...
                    latitude, longitude, ...
                    year, dayOfYear, UTseconds, ...
                    localApparentSolarTime, ...
                    f107avg, f107daily, magIdx );



                %species number densities
                n = rhovec(1) + rhovec(2) + rhovec(3) + rhovec(4) + ...
                    rhovec(5) + rhovec(7) + rhovec(8) + rhovec(9);   % number density

                rho6 = rhovec(6);

                m = rho6 / max(n,1e-30);   %kg

                %freestream temperature
                T_auto = Tvec(1);

                if contains(app.AtmosphereModeDropDown.Value, 'Manual', 'IgnoreCase', true)
                    T = app.AtmTEditField.Value;   %manual override
                else
                    %automatic model already provided from earlier
                    T = T_auto;
                end

                Tw = app.TwEditField.Value;
                %package into struct
                Atm.T         = T;
                Atm.rho_total = rho6;
                Atm.n_total   = n;
                Atm.m_avg     = m;

                atmosUsed = 'Automatic (NRLMSISE-00)';

            else
                %manual override 
                tbl = app.AtmosphereTable.Data;

                rho_species = cell2mat(tbl(:,2));   
                n_species   = cell2mat(tbl(:,3));   
                rho6 = sum(rho_species);            
                n    = sum(n_species);              
                T  = app.AtmTEditField.Value;
                Tw = app.AtmTwEditField.Value;
                m = rho6 / max(n,1e-35);

                Atm.T         = T;
                Atm.rho_total = rho6;
                Atm.n_total   = n;
                Atm.m_avg     = m;

                atmosUsed = 'Manual Override';
            end

            %molecular properties
            kB = 1.380649e-23;
            m  = rho6 / max(n,1e-35);   %no div zero

            Gr = 6.6743e-11;
            Me = 5.972e24;
            Re = 6371000;
            h  = app.AltitudeEditField.Value * 1000; %km to m

            if contains(app.VelocityModeDropDown.Value, 'Automatic', 'IgnoreCase', true)
                Vmag = sqrt(Gr * Me / (Re + h));
                U = [Vmag, 0, 0];
            else
                speed = app.VxEditField.Value;
                U = [speed, 0, 0];
            end
            Uhat = U ./ norm(U);

            %facet and panel properties

            %extract facets and unrotated vertices from loaded STL
            facets = stlobjectdata.ConnectivityList;
            unrotatedvertices = stlobjectdata.Points;
            N = size(facets, 1);

            %initialise struct fields
            panelproperties = struct( ...
                'vertices', [], ...           % rotated vertices
                'unrotatedvertices', [], ...  % original vertices
                'area', [], ...               % facet area
                'centroid', [], ...           % rotated centroid
                'ucentroid', [], ...          % unrotated centroid
                'normal', [], ...             % rotated normal
                'unormal', [], ...            % unrotated normal
                'epsilon_n', [], ...          % normal incidence cosine
                'epsilon_t', [], ...          % tangential component
                'tangent', [], ...            % unit tangent direction
                'isFF', [] );                 % forward-facing flag

            %loop over all facets
            for i = 1:N

                %extract indices of the three vertices in this facet
                indivertex = facets(i, :);

                %rotated vertices
                vertex_1 = vertices(indivertex(1), :);
                vertex_2 = vertices(indivertex(2), :);
                vertex_3 = vertices(indivertex(3), :);

                %unrotated vertices
                uvertex_1 = unrotatedvertices(indivertex(1), :);
                uvertex_2 = unrotatedvertices(indivertex(2), :);
                uvertex_3 = unrotatedvertices(indivertex(3), :);

                %store rotated and unrotated vertices exactly as your script does
                panelproperties(i).vertices           = [vertex_1; vertex_2; vertex_3];
                panelproperties(i).unrotatedvertices  = [uvertex_1; uvertex_2; uvertex_3];

                %facet edge lengths
                paneledge_1  = vertex_2 - vertex_1;
                paneledge_2  = vertex_3 - vertex_1;
                upaneledge_1 = uvertex_2 - uvertex_1;
                upaneledge_2 = uvertex_3 - uvertex_1;

                %facet area
                facetarea = norm(cross(paneledge_1, paneledge_2)) / 2;
                panelproperties(i).area = facetarea;

                %centroids (rotated and unrotated)
                facetcentroid  = (vertex_1  + vertex_2  + vertex_3 ) / 3;
                ufacetcentroid = (uvertex_1 + uvertex_2 + uvertex_3) / 3;

                panelproperties(i).centroid  = facetcentroid;
                panelproperties(i).ucentroid = ufacetcentroid;

                % Facet normals
                facetnormal     = cross(paneledge_1,  paneledge_2 );
                unrotatednormal = cross(upaneledge_1, upaneledge_2);

                facetnormal     = facetnormal     ./ norm(facetnormal);
                unrotatednormal = unrotatednormal ./ norm(unrotatednormal);

                panelproperties(i).normal  = facetnormal;
                panelproperties(i).unormal = unrotatednormal;

                epsilon_ni = dot(-U, facetnormal) / norm(U);
                panelproperties(i).epsilon_n = epsilon_ni;

                %tangential component of incidence
                epsilon_ti = sqrt(max(1 - (epsilon_ni^2), 0));
                panelproperties(i).epsilon_t = epsilon_ti;

                %facet tangent direction
                if sqrt((norm(U)^2) - (dot(U, facetnormal)^2)) > 1e-6
                    facettangent = (U - dot(U,facetnormal)*facetnormal) / ...
                        sqrt(norm(U)^2 - dot(U,facetnormal)^2);
                else
                    facettangent = [0,0,0];
                end

                panelproperties(i).tangent = facettangent;

            end
            app.PanelProps = panelproperties;


            %CG and unrotated ref length

            xcoord = unrotatedvertices(:,1);
            lunrefx = abs(max(xcoord) - min(xcoord));
            results.lunrefx = lunrefx;

            %CG method

            CG = [0, 0, 0];
            totalarea = 0;

            for i = 1:N
                facetcentroid = panelproperties(i).centroid;
                facetarea     = panelproperties(i).area;

                CG = CG + facetcentroid * facetarea;
                totalarea = totalarea + facetarea;
            end
            CG = CG / totalarea;
            results.CG = CG;


            %check if saved shadow data of current file and orientation exists
            app.StatusTextArea.Value = {'Checking for cached shadow data...'};

            if exist(shadowFilePath, 'file')
                load(shadowFilePath, 'BlockedPanels', 'BackwardPanels');
                fprintf('Loaded shadow data from %s\n', shadowFilePath);

                app.StatusTextArea.Value = { ...
                    'Cached shadow file found.'; ...
                    sprintf('Loaded: %s', shadowFilePath) ...
                    };

            else
                fprintf('No shadow data found, computing shadows...\n');

                app.StatusTextArea.Value = { ...
                    'No cached shadow file found.'; ...
                    'Computing shadows, ray-tracing...' ...
                    };

                %%%START OF RAY-TRACING ALGORITHM%%%
                %%%FORWARD OR BACKWARDS FACING PANEL CHECK%%%
                BlockedPanels = false(N, 1);
                BackwardPanels = false(N, 1);
                for i = 1:N
                    epsilon_ni = panelproperties(i).epsilon_n;
                    %classify if a panel is forward or backwards facing
                    if epsilon_ni > -1e-1
                        panelproperties(i).isFF = true;
                    else
                        panelproperties(i).isFF = false;
                        BackwardPanels(i) = true;
                    end
                end

                %%%RAY CASTING FROM EACH FF PANELS CENTROID%%%
                %casting a ray from each FF panel in the -ve x direction
                parfor i = 1:N 
                    %only use FF panels
                    if panelproperties(i).isFF
                        %the centroid of the current panel is the rays origin
                        rayO = panelproperties(i).centroid + (1e-3 * panelproperties(i).normal); %offset the origin of the cast ray along the normal to avoid intersection with parallel facets
                        %ray casts to the -ve x direction
                        raydir = [-1; 0; 0];

                        %intersection check
                        isBlocked = false;
                        for j = 1:N
                            if i ~= j %this skips checking the same panel and checks only with FF panels
                                %skip panels that are upstream
                                relativePosition = panelproperties(j).centroid - rayO;
                                if dot(relativePosition, Uhat) > 0
                                    continue;
                                end
                                %j-th panel vertices extraction
                                vertex_1 = panelproperties(j).vertices(1, :)';
                                vertex_2 = panelproperties(j).vertices(2, :)';
                                vertex_3 = panelproperties(j).vertices(3, :)';
                                %check for intersection using MT algo
                                intersect = rayfunction(rayO', raydir, vertex_1, vertex_2, vertex_3);
                                %for a detection of an intersection mark as blocked
                                if intersect
                                    isBlocked = true;
                                    break; %stop checking for further blocking
                                end
                            end
                        end
                        %mark as blocked if intersection found
                        if isBlocked
                            BlockedPanels(i) = true;
                        end
                    end
                end

                %saving computed shadow data for future use
                save(shadowFilePath, 'BlockedPanels', 'BackwardPanels', 'psi', 'theta', 'phi');
                fprintf('Saved shadow data to %s\n', shadowFilePath);

                app.StatusTextArea.Value = { ...
                    'Ray-tracing completed.'; ...
                    sprintf('Saved shadow file: %s', shadowFilePath) ...
                    }; 
            end
            %%%END OF RAY-TRACING ALGORITHM%%%



            %store
            app.PanelProps     = panelproperties;
            app.BlockedPanels  = BlockedPanels;
            app.BackwardPanels = BackwardPanels;

            results.NPanels        = N;
            results.NBlocked       = nnz(BlockedPanels);
            results.NBackward      = nnz(BackwardPanels);
            results.shadowFilePath = shadowFilePath;

            %begin force and moment calc

            %constants and variables
            T      = Atm.T;
            rho6   = Atm.rho_total;
            m      = Atm.m_avg;
            Tw     = app.TwEditField.Value;
            Uhat   = U./norm(U);
            an     = app.AlphaNEditField.Value;
            at     = app.AlphaTEditField.Value;


            %FF and unblocked
            contributingPanels = find(~BlockedPanels & ~BackwardPanels);

            %initialise force and moment
            F_rho   = [0,0,0];
            Fp_rho  = [0,0,0];
            Ftau_rho= [0,0,0];
            M_rho   = [0,0,0];

            %gas speeds
            Vg = sqrt((2*kB*T)/m);
            Vw = sqrt((2*kB*Tw)/m);

            %speed ratio
            S = norm(U)/Vg;

            %force and moment script
            for i = contributingPanels'
                facetcentroid = panelproperties(i).centroid;
                facetnormal   = panelproperties(i).normal;
                facettangent  = panelproperties(i).tangent;
                epsilon_ni    = panelproperties(i).epsilon_n;
                epsilon_ti    = panelproperties(i).epsilon_t;
                facetarea     = panelproperties(i).area;

                %Terms
                s     = S * epsilon_ni;
                es    = erf(s);
                exps2 = exp(-(s^2));
                G1    = (1/sqrt(pi)) * ( (s*exps2) + ((sqrt(pi)/2)*(1+(2*(s^2)))*(1 + es)) );
                G2    = (1/sqrt(pi)) * ( exps2 + (sqrt(pi)*s*(1 + es)) );

                %Cercignani-Lampis-Lord GSIM model
                pi_rho = 0.5*((norm(U)/S)^2)*G1;

                if an == 0
                    pr_rho = pi_rho; %specular reflection to avoid errors
                else
                    c  = sqrt((1-an)/(2*an)) * sqrt(T/Tw);
                    t1 = sqrt(1-an)*G1;
                    t2 = (sqrt(an)/2)*(Vw/Vg);

                    A1 = 0.907;  A2 = 1.425;  A3 = -1.332;
                    B1 = 1.876;  B2 = 0.0963; B3 = 0.0963;

                    t3 = (A1*(s - (B1*c)/2)*exp(((B1*c)^2)/4 - B1*c*s)*erfc((B1*c)/2 - s)) + ...
                        (A2*(s - (B2*c)/2)*exp(((B2*c)^2)/4 - B2*c*s)*erfc((B2*c)/2 - s)) + ...
                        (A3*(s - (B3*c)/2)*exp(((B3*c)^2)/4 - B3*c*s)*erfc((B3*c)/2 - s));

                    pr_rho = 0.5*((norm(U)/S)^2) * ( t1 + t2*(exps2 + sqrt(pi)*t3) );
                end

                taui_rho = 0.5*((norm(U)/S)^2) * S * epsilon_ti * G2;
                taur_rho = taui_rho * sqrt(1-at);

                %forces
                Fpi_rho = (pi_rho + pr_rho)*(-facetnormal)*facetarea;
                Ftaui_rho = (taui_rho - taur_rho)*facettangent*facetarea;
                Fi_rho = Fpi_rho + Ftaui_rho;


                p_incident(i)  = pi_rho  * rho6;
                p_reflect(i)   = pr_rho  * rho6;
                tau_s(i)       = (taui_rho - taur_rho) * rho6;

                F_rho = F_rho + Fi_rho;
                Ftau_rho = Ftau_rho + Ftaui_rho;
                Fp_rho = Fp_rho + Fpi_rho;

                %moments
                %moment arm from each panel centroid to CG
                r = facetcentroid - CG;
                Mi_rho = cross(r, Fi_rho);
                M_rho = M_rho + Mi_rho;
            end

            %unrotated contrib ref area
            shadowFileName_0 = sprintf('%s_ps0_th0_ph0.mat', stlName);
            shadowFilePath_0 = fullfile(app.ShadowFolder, shadowFileName_0);

            if exist(shadowFilePath_0, 'file')
                load(shadowFilePath_0, 'BlockedPanels', 'BackwardPanels');
                Aunref = 0;
                contributingPanels_0 = find(~BlockedPanels & ~BackwardPanels);

                for i = contributingPanels_0'
                    unrotatedfacetnormal = panelproperties(i).unormal;
                    projection = abs(dot(unrotatedfacetnormal, Uhat));
                    if projection > -1e-3
                        unrotatedprojectedArea = panelproperties(i).area * projection;
                        Aunref = Aunref + unrotatedprojectedArea;
                    end
                end
            else
                error('Shadow data for psi=theta=phi=0 does not exist. Run with zero rotation first.');
            end

            %force coeff
            CD  = dot(F_rho, Uhat)/(0.5*Aunref*(norm(U)^2));
            CDp = dot(Fp_rho, Uhat)/(0.5*Aunref*(norm(U)^2));
            CDtau = dot(Ftau_rho, Uhat)/(0.5*Aunref*(norm(U)^2));

            CL  = dot(F_rho, [0,0,1])/(0.5*Aunref*(norm(U)^2));
            CLp = dot(Fp_rho, [0,0,1])/(0.5*Aunref*(norm(U)^2));
            CLtau = dot(Ftau_rho, [0,0,1])/(0.5*Aunref*(norm(U)^2));

            CS  = dot(F_rho, [0,1,0])/(0.5*Aunref*(norm(U)^2));
            CSp = dot(Fp_rho, [0,1,0])/(0.5*Aunref*(norm(U)^2));
            CStau = dot(Ftau_rho, [0,1,0])/(0.5*Aunref*(norm(U)^2));

            CAb = dot(F_rho, (R_ypr*[1,0,0]')')/(0.5*Aunref*(norm(U)^2));
            CSb = dot(F_rho, (R_ypr*[0,1,0]')')/(0.5*Aunref*(norm(U)^2));
            CNb = dot(F_rho, (R_ypr*[0,0,1]')')/(0.5*Aunref*(norm(U)^2));

            Cl = dot(M_rho, (R_ypr*[1,0,0]')')/(0.5*Aunref*lunrefx*(norm(U)^2));
            Cm = dot(M_rho, (R_ypr*[0,1,0]')')/(0.5*Aunref*lunrefx*(norm(U)^2));
            Cn = dot(M_rho, (R_ypr*[0,0,1]')')/(0.5*Aunref*lunrefx*(norm(U)^2));

            %store into results struc
            results.CD      = CD;
            results.CDp     = CDp;
            results.CDtau   = CDtau;
            results.CL      = CL;
            results.CLp     = CLp;
            results.CLtau   = CLtau;
            results.CS      = CS;
            results.CSp     = CSp;
            results.CStau   = CStau;

            results.Cl      = Cl;
            results.Cm      = Cm;
            results.Cn      = Cn;

            %3d visual pressure and shear
            results.Pressure.incident   = p_incident(:);
            results.Pressure.reflected  = p_reflect(:);
            results.Pressure.shear      = tau_s(:);

            results.CAb     = CAb;
            results.CSb     = CSb;
            results.CNb     = CNb;

            results.FD      = CD*rho6;
            results.FL      = CL*rho6;
            results.FS      = CS*rho6;

            results.CG      = CG;
            results.lunrefx = lunrefx;
            results.Aunref  = Aunref;

            results.Umag    = norm(U);
            results.Sratio  = S;
            results.T       = T;
            results.rho     = rho6;
            results.m       = m;

            %end of force calc
            app.LastResults = results;

        end

        function plotShadows(app)
            if isempty(app.STLData) || isempty(app.RotatedVerts) ...
                    || isempty(app.BlockedPanels) || isempty(app.BackwardPanels)
                return;
            end

            faces = app.STLData.ConnectivityList;
            verts = app.RotatedVerts;
            N     = size(faces, 1);

            %1 is unshadowed, 2 is shadowed (blocked or backward)
            cdata = ones(N, 1);

            %shadowed if either blocked OR backward-facing
            shadowed = false(N,1);
            if ~isempty(app.BlockedPanels)
                shadowed = shadowed | app.BlockedPanels;
            end
            if ~isempty(app.BackwardPanels)
                shadowed = shadowed | app.BackwardPanels;
            end

            cdata(shadowed) = 2;             %shadowed panels

            cla(app.ShadowAxes);

            patch(app.ShadowAxes, ...
                'Faces', faces, ...
                'Vertices', verts, ...
                'FaceVertexCData', cdata, ...
                'FaceColor', 'flat', ...
                'EdgeColor', 'none');

            axis(app.ShadowAxes, 'equal');
            grid(app.ShadowAxes, 'on');
            view(app.ShadowAxes, 3);
            xlabel(app.ShadowAxes, 'X [m]');
            ylabel(app.ShadowAxes, 'Y [m]');
            zlabel(app.ShadowAxes, 'Z [m]');

            colormap(app.ShadowAxes, [ ...
                0.3 0.5 0.9;
                0.3 0.3 0.3]);
            caxis(app.ShadowAxes, [1 2]);

            hold(app.ShadowAxes, 'on');

            h1 = patch(app.ShadowAxes, nan, nan, [0.3 0.5 0.9], ...
                'DisplayName', 'Molecule impinge');
            h2 = patch(app.ShadowAxes, nan, nan, [0.3 0.3 0.3], ...
                'DisplayName', 'Molecule not impinge');

            lg = legend(app.ShadowAxes, [h1 h2], 'Location', 'southoutside');
            lg.FontSize = 14;
            lg.ItemTokenSize = [20 18];
            hold(app.ShadowAxes, 'on');


            verts = app.RotatedVerts;
            xmin = min(verts(:,1));
            xmax = max(verts(:,1));
            ymin = min(verts(:,2));
            ymax = max(verts(:,2));
            zmin = min(verts(:,3));
            zmax = max(verts(:,3));

            Lx = xmax - xmin;
            behind_offset = 0.5 * Lx;

            arrow_start = [xmax + behind_offset, ...
                (ymin + ymax)/2, ...
                (zmin + zmax)/2];
            arrow_direction = [0.3 * Lx, 0, 0];

            quiver3(app.ShadowAxes, ...
                arrow_start(1), arrow_start(2), arrow_start(3), ...
                arrow_direction(1), arrow_direction(2), arrow_direction(3), ...
                0, ...
                'LineWidth', 2, ...
                'MaxHeadSize', 1.5, ...
                'Color', [0.3 0.5 0.9], ...
                'DisplayName', 'Free-stream Direction');

            text(app.ShadowAxes, ...
                arrow_start(1) + arrow_direction(1)*1.1, ...
                arrow_start(2), ...
                arrow_start(3), ...
                'Free-stream Direction', ...
                'FontWeight', 'bold');
            set(app.ShadowAxes, 'XMinorTick', 'on', 'YMinorTick', 'on');

            hold(app.ShadowAxes, 'off');
        end

        %pressure plotting
        function plotPressure(app)
            %results of pressure/stress
            if isempty(app.STLData) || isempty(app.RotatedVerts) || isempty(app.LastResults)
                cla(app.PressureAxes);
                title(app.PressureAxes, 'Pressure / Stress Distribution');
                return;
            end

            faces = app.STLData.ConnectivityList;
            verts = app.RotatedVerts;
            R     = app.LastResults;

            switch app.QuantityDropDown.Value

                case 'Incident pressure [N]'
                    cdata = R.Pressure.incident;

                case 'Reflected pressure [N]'
                    cdata = R.Pressure.reflected;

                case 'Shear stress [N]'
                    cdata = R.Pressure.shear;

                otherwise
                    cdata = zeros(size(faces,1),1);
            end

            Nfaces = size(faces,1);
            if numel(cdata) < Nfaces
                tmp = zeros(Nfaces,1);
                tmp(1:numel(cdata)) = cdata;
                cdata = tmp;
            end

            %panel masking
            mask = ~app.BlockedPanels & ~app.BackwardPanels;
            cdata_masked = zeros(Nfaces,1);
            cdata_masked(mask) = cdata(mask);

            cdata = cdata_masked;

            %surface plot
            cla(app.PressureAxes);

            patch(app.PressureAxes, ...
                'Faces', faces, ...
                'Vertices', verts, ...
                'FaceVertexCData', cdata, ...
                'FaceColor', 'flat', ...
                'EdgeColor', 'none');

            axis(app.PressureAxes, 'equal');
            grid(app.PressureAxes, 'on');
            view(app.PressureAxes, 3);

            xlabel(app.PressureAxes, 'X [m]');
            ylabel(app.PressureAxes, 'Y [m]');
            zlabel(app.PressureAxes, 'Z [m]');

            colormap(app.PressureAxes, 'jet');
            cb = colorbar(app.PressureAxes);
            cb.Label.String = app.QuantityDropDown.Value;

            %gradient colour
            nz = cdata(cdata ~= 0);
            if ~isempty(nz)
                caxis(app.PressureAxes, [min(nz) max(nz)]);
            else
                caxis(app.PressureAxes, [0 1]);
            end
            title(app.PressureAxes, sprintf('Distribution: %s', app.QuantityDropDown.Value));
        end
    end

    %initialisation and deletion for app
    methods (Access = public)

        function app = LEOAEROSIMbyAliAerospace
            createComponents(app);
            registerApp(app, app.UIFigure);

            app.ShadowFolder = fullfile('/Users/ali./Documents/MATLAB', 'MATLAB RAYTRACING');
            if ~exist(app.ShadowFolder, 'dir')
                mkdir(app.ShadowFolder);
            end

            %initialise
            updateAutoVelocity(app);
            updateInputOverview(app);
            updateComputedOverview(app);

            if nargout == 0
                clear app
            end
        end

        %remove
        function delete(app)
            if isvalid(app.UIFigure)
                delete(app.UIFigure);
            end
        end
    end
end