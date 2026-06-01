% DESCRIPTION: Dynamic control loop module. Executes VISSIM COM interface, real-time
%              telemetry, and Extended Green Wave (SP) / Red Line (SC) priority algorithms.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

%% 0. Initialization and VISSIM connection
% Launch the VISSIM COM server and load the network file
feature('COM_SafeArraySingleDim', 1);
Vissim = actxserver('Vissim.Vissim');
[networkName, networkPath] = uigetfile('*.inpx');
Vissim.LoadNet(fullfile(networkPath, networkName), false);

%% 1. Network data extraction (static infrastructure)
% Extract base network geometry (executed once)
run('Import_network_data.m'); 

% Interpolate X,Y coordinates of each signal head for real-distance calculations
SignalHeadsCoords = Coords_of_signal_heads(NoOfLinks, NoOfSignalHeads, LinkOfSignalHeads, PosOfSignalHeads, CoordsOfLinks, Length2DOfLinks);

%% 2. Topological processing and analytical variables
% Geometric tolerances for the navigation graph
ConnectionTolerance = 1.0;  % [m] margin for connections between links
ParallelTolerance = 0.5;    % [m] margin for grouping parallel signal heads

% Link-level adjacency lists for graph navigation
[LinksAdjacencyList, LinksConnectionPoints] = Adjacency_list_of_links(NoOfLinks, FromLinkOfLinks, ToLinkOfLinks, FromPosOfLinks, ToPosOfLinks);

% Signal head adjacency list (control graph)
[SignalHeadsAdjacencyList, SignalHeadsDistances] = Adjacency_list_of_real_signal_heads(NoOfLinks, NoOfSignalHeads, LinkOfSignalHeads, SignalHeadsOfLinks, FromLinkOfLinks, ToLinkOfLinks, IsConnOfLinks, FromPosOfLinks, ToPosOfLinks, PosOfSignalHeads, Length2DOfLinks, ParallelTolerance, ConnectionTolerance);

% Mean civilian traffic speed for conflict ETA computation
MeanSpeedKmh = 40; % [km/h] average urban traffic speed (adjustable per scenario)

% Topological conflict list between signal head groups
[SignalHeadsConflictList, SignalHeadsConflictDistances, SignalHeadsConflictETAs] = Conflict_list_of_signal_heads(LinksAdjacencyList, NoOfLinks, LinkOfSignalHeads, NoOfSignalHeads, PosOfSignalHeads, SignalHeadsOfLinks, Length2DOfLinks, CoordsOfLinks,FromLinkOfLinks, ToLinkOfLinks, IsConnOfLinks, FromPosOfLinks, ToPosOfLinks, LinksConnectionPoints, ConnectionTolerance, ParallelTolerance, MeanSpeedKmh);

% Infrastructure factor (turn penalties and geometric complexity)
InfrastructureFactor = Calculate_infrastructure_factor(NoOfLinks, NumLanesOfLinks, Length2DOfLinks, CoordsOfLinks, LinksAdjacencyList);

% Identification of decision links (bifurcation points)
IsDecisionLink = Identify_decision_links(NoOfLinks, LinksAdjacencyList);

%% 3. Dynamic factors and scenario configuration
WeatherFactor = 1.0;
VisibilityFactor = 1.0;
PedestrianRiskFactor = 1.0;
CriticalDensity = 25; % [veh/km] saturation traffic density estimate. Adjust per link based on actual capacity.
DischargeVelocity = 10; % [m/s]
IncidentResistance = zeros(1, length(NoOfLinks));

% --- Critical control parameters ---
DesiredSpeedVE_Kmh = 70;                   % EV target speed [km/h]
V_Desired = DesiredSpeedVE_Kmh / 3.6;      % Mandatory conversion to [m/s]

% Temporal safety margin (seconds) added to link clearance time
SafetyMargin = 3.0;
% Recalculation period: maximum time between trigger updates (seconds)
% (Future: dynamic trigger driven by information density)
RecalculationPeriod = 5.0;

% BFS internal capacities for Calculate_sp (adjustable for large networks)
BFS_QueueCapacity = 256;    % Initial BFS queue capacity
BFS_QueueGrowth = 256;      % Queue growth increment
BFS_SPCapacity = 256;       % Initial SP result vector capacity

% State transition time (v1 simplification: fixed value instead of function)
TransitionTime = 1.0; % [s] delay between state changes (normal->forced and forced->normal)

TrafficLevels = struct('FREE_FLOW', 0.4, 'LIGHT', 0.7, 'MODERATE', 1.0, 'HEAVY', 1.3, 'SEVERE', 1.6, 'EXTREME', 2.0);
CongestionLevel = 'MODERATE'; 
Phi = TrafficLevels.(CongestionLevel);

% Scale original VehicleInputs by the congestion factor Phi
[OriginalTrafficVolumes] = Configure_traffic_volume(Vissim, CongestionLevel);

% Lambda (elastic resistance) in coherent units [s/m]
LambdaOfLinks = ((Phi .* CriticalDensity .* Length2DOfLinks) ./ (double(NumLanesOfLinks) .* DischargeVelocity)) + IncidentResistance;

% Base clearance time per link (seconds)
ClearanceTimeOfLinks = (Length2DOfLinks ./ V_Desired) + (Length2DOfLinks .* LambdaOfLinks);

%% 4. Warm-up phase (turbo mode) and dynamic extraction
WarmUpSeconds = 300;    % Warm-up seconds before EV insertion

% Configure fast time-skip for warm-up
Vissim.Graphics.set('AttValue', 'QuickMode', 1);
Vissim.Simulation.set('AttValue', 'UseMaxSimSpeed', true);
Vissim.Simulation.set('AttValue', 'SimBreakAt', WarmUpSeconds);

% Run continuous warm-up until the defined second
Vissim.Simulation.RunContinuous(); 

% RESTORE SETTINGS FOR REAL-TIME PRIORITIZATION
% (Essential to see EV flashing and signal head state changes)
Vissim.Graphics.set('AttValue', 'QuickMode', 0);
Vissim.Simulation.set('AttValue', 'UseMaxSimSpeed', false);
Vissim.Simulation.set('AttValue', 'SimSpeed', 1.0); % Real Speed
Vissim.Simulation.set('AttValue', 'SimBreakAt', 0); % Next break at end

% SINGLE EXTRACTION POINT: extract cycle times after warm-up
% Data now reflects the already-congested network state
run('Import_cycle_times_signal_heads_data.m');

%% 5. Emergency Vehicle (EV) insertion and configuration
% Creation parameters
EmergencyVehicleName = 'Ambulance';
EmergencyVehicleInitialLink = NoOfLinks(~IsConnOfLinks); % First regular link (not connector)
EmergencyVehicleInitialLink = EmergencyVehicleInitialLink(1);
EmergencyVehiclePrimaryColor = 'FF0000'; % Red (RGB: 0x0000FF)
EmergencyVehicleSecondaryColor = 'ffffff'; % White (RGB: 0xFFFFFF)
EmergencyVehicleInteraction = true; % Interaction with civilian traffic enabled
ZoomRangeMin = 50;   % Base zoom range (focused on EV)
ZoomRangeMax = 100;  % Maximum range to encompass the Extended Green Wave

% Insert the EV via external function
EmergencyVehicleObject = Add_emergency_vehicle(Vissim, EmergencyVehicleName, DesiredSpeedVE_Kmh, EmergencyVehicleInitialLink, EmergencyVehiclePrimaryColor, EmergencyVehicleInteraction);

% Initial route configuration (deterministic routing)
LinkIDToIndex = containers.Map(num2cell(NoOfLinks), num2cell(1:numel(NoOfLinks)));
InitialNextLink = Choose_min_downstream_link(EmergencyVehicleInitialLink, 0, LinksAdjacencyList, LinksConnectionPoints, IsConnOfLinks, LinkIDToIndex, ConnectionTolerance);

if ~isnan(InitialNextLink)
    Try_set_vehicle_desired_link(EmergencyVehicleObject, InitialNextLink);
end

%% 6. Simulation loop and dynamic prioritization
LastEVLink = 0;
SP_Current = []; 
SC_Current = []; 
FlashCounter = 0;
FlashState = true;
PrevETAToSH = NaN;   % NaN = no valid previous measurement (Has_ETA_converged ignores it)
LastEVPos = Inf;     % EV position in previous step (signal head crossing detection)
LastRecalcTime = 0;  % Time of last recalculation (s)
PendingUpdate = struct('Active', false, 'ApplyAtTime', 0, 'SP', [], 'SC', []);

% --- O(1) lookup arrays (precomputed once) ---
max_link_id_eps = max(NoOfLinks);
LinkLookup = zeros(1, max_link_id_eps);
LinkLookup(NoOfLinks) = 1:numel(NoOfLinks);
max_sh_id_eps = max(NoOfSignalHeads);
SHLookup = zeros(1, max_sh_id_eps);
SHLookup(NoOfSignalHeads) = 1:numel(NoOfSignalHeads);

EmergencyVehicleID = EmergencyVehicleObject.AttValue('No');
EmergencyVehicleExists = true; % Control flag initialization

% Engineering telemetry (SI units)
PREALLOCATED_STEPS = 36000;
EmergencyMetrics = struct('StartTime', Vissim.Simulation.AttValue('SimSec'), 'Speeds', zeros(1, PREALLOCATED_STEPS), 'TotalDistance', 0);
SpeedCount = 0;

% Maximum safety simulation time: 3600 seconds
MaxSimTime = 3600;

% Main simulation cycle with dynamic prioritization
while (Vissim.Simulation.AttValue('SimSec') < MaxSimTime) && EmergencyVehicleExists
    
    % 1. Advance simulation (the EV moves physically here)
    Vissim.Simulation.RunSingleStep();

    % 2. Verify whether the EV is still present in the network
    try
        EmergencyVehicleExists = Vissim.Net.Vehicles.ItemByKey(EmergencyVehicleID);
        EmergencyVehicleExists = ~isempty(EmergencyVehicleExists);
    catch
        EmergencyVehicleExists = false;
    end

    % If the EV has left the network, exit the loop without further processing
    if ~EmergencyVehicleExists, break; end
    
    try
        % A. EV physical state via VISSIM COM
        LaneAttr = EmergencyVehicleObject.AttValue('Lane'); 
        if isempty(LaneAttr), continue; end % Skip if attribute temporarily fails
        
        C_Attr = strsplit(LaneAttr, '-');
        CurrentEVLink = str2double(C_Attr{1});
        CurrentEVPos  = EmergencyVehicleObject.AttValue('Pos');
        CurrentEVSpeed_Kmh = EmergencyVehicleObject.AttValue('Speed'); 

        % Reset LastEVPos on link change (positions are not comparable across links)
        if CurrentEVLink ~= LastEVLink
            LastEVPos = Inf;
        end
        
        SpeedCount = SpeedCount + 1;
        EmergencyMetrics.Speeds(SpeedCount) = CurrentEVSpeed_Kmh;
        
        % B. VISUAL FEEDBACK AND ADAPTIVE ZOOM
        FlashCounter = FlashCounter + 1;
        if mod(FlashCounter, 3) == 0
            % Toggle flash state and apply corresponding color
            FlashState = ~FlashState;
            FlashCounter = 0;
            if FlashState
                set(EmergencyVehicleObject, 'AttValue', 'Color1', EmergencyVehiclePrimaryColor);
            else
                set(EmergencyVehicleObject, 'AttValue', 'Color1', EmergencyVehicleSecondaryColor);
            end
            
            % Fixed zoom centered on the EV (±50 units)
            CoordStr = EmergencyVehicleObject.AttValue('CoordFront');
            if ischar(CoordStr)
                EVCoords = str2num(CoordStr); %#ok<ST2NM>
                if ~isempty(EVCoords)
                    Vissim.Graphics.CurrentNetworkWindow.ZoomTo(EVCoords(1) - 50, EVCoords(2) - 50, EVCoords(1) + 50, EVCoords(2) + 50);
                end
            end
        end

        % C. EVENT TRIGGER STATE MACHINE
        % Pre-compute ETA and clearance time once per step (O(1) lookup)
        curr_link_idx_eta = LinkLookup(CurrentEVLink);
        L_curr = Length2DOfLinks(curr_link_idx_eta);
        lambda_curr = LambdaOfLinks(curr_link_idx_eta);
        T_Despeje_Current = (L_curr / V_Desired) + (L_curr * lambda_curr);
        if ~isempty(SP_Current)
            dist_eta = Calculate_distance_to_sh(CurrentEVLink, CurrentEVPos, SP_Current(1), Length2DOfLinks, PosOfSignalHeads, LinkOfSignalHeads, SP_Current, SignalHeadsDistances, SHLookup, LinkLookup);
            CurrentETAToSH = (dist_eta / V_Desired) + (dist_eta * lambda_curr);
        else
            CurrentETAToSH = Inf;
        end

        SimTime = Vissim.Simulation.AttValue('SimSec');

        % Apply pending transition when the delay has expired
        if PendingUpdate.Active && SimTime >= PendingUpdate.ApplyAtTime
            Update_infrastructure_states(Vissim, SP_Current, SC_Current, PendingUpdate.SP, PendingUpdate.SC, NoOfSignalHeads, SCOfSignalHeads, SGOfSignalHeads);
            SP_Current = PendingUpdate.SP;
            SC_Current = PendingUpdate.SC;
            PendingUpdate.Active = false;
        end

        [triggered, triggerType] = Check_recalculation_trigger(CurrentEVLink, LastEVLink, CurrentEVPos, LastEVPos, IsDecisionLink, SP_Current, PosOfSignalHeads, LinkOfSignalHeads, CurrentETAToSH, PrevETAToSH, T_Despeje_Current, SimTime, LastRecalcTime, RecalculationPeriod, LinkLookup, SHLookup);
        if (LastEVLink == 0) || triggered
            if LastEVLink == 0
                triggerType = 'INIT';
            end

            % 1. Extended Green Wave (SP) with dynamic horizon: cumulative_ETA <= (clearance_time + SafetyMargin)
            SP_New = Calculate_sp(SignalHeadsAdjacencyList, SignalHeadsDistances, NoOfLinks, NoOfSignalHeads, SignalHeadsOfLinks, PosOfSignalHeads, Length2DOfLinks, ConnectionTolerance, CurrentEVLink, CurrentEVPos, ClearanceTimeOfLinks, SafetyMargin, DesiredSpeedVE_Kmh, BFS_QueueCapacity, BFS_QueueGrowth, BFS_SPCapacity);
            
            % 2. Red Line (SC): signal heads in conflict with the Extended Green Wave -> RED
            SC_New = Calculate_sc(SignalHeadsConflictList, NoOfSignalHeads, SP_New);
            LastRecalcTime = SimTime;

            % 3. Display prioritized and conflicting signal heads
            fprintf('[t=%.1fs] %s | Link=%d Pos=%.1f | SP=[%s] (%d) | SC=[%s] (%d)\n', SimTime, triggerType, CurrentEVLink, CurrentEVPos, num2str(SP_New), numel(SP_New), num2str(SC_New), numel(SC_New));

            % 4. Schedule state transition (delayed application by TransitionTime seconds)
            if ~isequal(sort(SP_Current), sort(SP_New)) || ~isequal(sort(SC_Current), sort(SC_New))
                PendingUpdate = struct('Active', true, 'ApplyAtTime', SimTime + TransitionTime, 'SP', SP_New, 'SC', SC_New);
            end

            % STATE MEMORY UPDATE (SP_Current/SC_Current updated when transition is applied)
            LastEVLink = CurrentEVLink;
            LastEVPos = CurrentEVPos;
            PrevETAToSH = NaN;  % NaN = no valid previous data; Has_ETA_converged ignores it
        else
            LastEVPos = CurrentEVPos;  % Advance position for crossing detection
            PrevETAToSH = CurrentETAToSH;  % Advance state for edge detection
        end
        
    catch ME
        % If the error is due to EV disappearance, exit cleanly
        if contains(ME.message, 'Invalid') || contains(ME.message, 'not found')
            break;
        end
        fprintf('Simulation warning: %s\n', ME.message);
    end
end

%% 7. Finalization and infrastructure cleanup
fprintf('Simulation complete. Releasing infrastructure control...\n');

% Apply any pending transition before releasing (if left pending on loop exit)
if PendingUpdate.Active
    Update_infrastructure_states(Vissim, SP_Current, SC_Current, PendingUpdate.SP, PendingUpdate.SC, NoOfSignalHeads, SCOfSignalHeads, SGOfSignalHeads);
    SP_Current = PendingUpdate.SP;
    SC_Current = PendingUpdate.SC;
end

if ~isempty(SP_Current) || ~isempty(SC_Current)
    Update_infrastructure_states(Vissim, SP_Current, SC_Current, [], [], NoOfSignalHeads, SCOfSignalHeads, SGOfSignalHeads);
end

% Final EV telemetry
EmergencyMetrics.Speeds = EmergencyMetrics.Speeds(1:SpeedCount);
save('Emergency_Run_Results.mat', 'EmergencyMetrics');
