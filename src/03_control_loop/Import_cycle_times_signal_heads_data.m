% DESCRIPTION: Extracts signal controller cycle times from the VISSIM COM interface
%              after the warm-up phase. Computes per-signal-head transition times
%              and packs them into the SignalHeadsData struct.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

% Extract signal controller attributes via a single COM call
SignalControllersObj = Vissim.Net.SignalControllers;
SCAttrs = SignalControllersObj.GetMultipleAttributes({'No'; 'CycTm'});

% Controller variables (visible in Workspace)
in_no_of_signal_controllers = cell2mat(SCAttrs(:,1))';
in_cycle_time_of_controllers = cell2mat(SCAttrs(:,2))';

% Map each signal head's controller ID to its index in the controller array,
% then assign the corresponding cycle time as the transition time
[~, sc_idx_map] = ismember(SCOfSignalHeads, in_no_of_signal_controllers);
SignalHeadsTransitionTimes = in_cycle_time_of_controllers(sc_idx_map);

% Safety margin: replace zero or negative cycle times with a default (Amber + AllRed)
SignalHeadsTransitionTimes(SignalHeadsTransitionTimes <= 0) = 5.0;

% --- SignalHeadsData packing ---
SignalHeadsData.TransitionTimes = SignalHeadsTransitionTimes;
SignalHeadsData.ControllerIDs = in_no_of_signal_controllers;
SignalHeadsData.ControllerCycles = in_cycle_time_of_controllers;

clear SignalControllersObj SCAttrs in_no_of_signal_controllers in_cycle_time_of_controllers sc_idx_map;
