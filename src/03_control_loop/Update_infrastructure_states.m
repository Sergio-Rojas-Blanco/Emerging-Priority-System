% DESCRIPTION: Synchronizes signal head states in VISSIM via COM. Releases control
%              of old signal heads and applies new Extended Green Wave (SP) and
%              Red Line (SC) states based on Signal Group identification.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function Update_infrastructure_states(in_vissim, in_sp_old, in_sc_old, in_sp_new, in_sc_new, in_no_of_signal_heads, in_sc_of_signal_heads, in_sg_of_signal_heads)
% UPDATE_INFRASTRUCTURE_STATES Synchronizes signal head states in VISSIM via COM.
% Releases control of old signal heads and applies new priority (GREEN) and
% conflict (RED) states based on Signal Group identification.
%
% INPUTS:
% - in_vissim: VISSIM COM interface object.
% - in_sp_old: Row vector with VISSIM IDs of the previous Extended Green Wave (SP).
% - in_sc_old: Row vector with VISSIM IDs of the previous Red Line (SC).
% - in_sp_new: Row vector with VISSIM IDs of the new Extended Green Wave (SP).
% - in_sc_new: Row vector with VISSIM IDs of the new Red Line (SC).
% - in_no_of_signal_heads: Row vector with all signal head VISSIM IDs.
% - in_sc_of_signal_heads: Row vector with the controller ID per signal head [sh_idx].
% - in_sg_of_signal_heads: Row vector with the signal group ID per signal head [sh_idx].

% 1. IDENTIFY SIGNAL HEADS TO RELEASE
% Those in the old lists but not in the new lists
old_combined = unique([in_sp_old, in_sc_old], 'stable');
new_combined = unique([in_sp_new, in_sc_new], 'stable');
release_ids = setdiff(old_combined, new_combined, 'stable');

% Release COM control of signal heads no longer participating in prioritization
for i = 1:length(release_ids)
    [~, sh_idx] = ismember(release_ids(i), in_no_of_signal_heads);
    sc_id = in_sc_of_signal_heads(sh_idx);
    sg_id = in_sg_of_signal_heads(sh_idx);
    % Access the Signal Group object via Controller -> SignalGroup hierarchy
    sg_obj = in_vissim.Net.SignalControllers.ItemByKey(sc_id).SGs.ItemByKey(sg_id);
    % Return control to the normal signal plan
    sg_obj.set('AttValue', 'ContrByCOM', false);
end

% 2. APPLY RED LINE (SC_NEW)
for i = 1:length(in_sc_new)
    [~, sh_idx] = ismember(in_sc_new(i), in_no_of_signal_heads);
    sc_id = in_sc_of_signal_heads(sh_idx);
    sg_id = in_sg_of_signal_heads(sh_idx);
    sg_obj = in_vissim.Net.SignalControllers.ItemByKey(sc_id).SGs.ItemByKey(sg_id);
    
    % Force COM control and set state to RED
    sg_obj.set('AttValue', 'ContrByCOM', true);
    sg_obj.set('AttValue', 'SigState', 'RED');
end

% 3. APPLY EXTENDED GREEN WAVE (SP_NEW)
% Applied last so that, in case of logic conflict, the EV's GREEN prevails
for i = 1:length(in_sp_new)
    [~, sh_idx] = ismember(in_sp_new(i), in_no_of_signal_heads);
    sc_id = in_sc_of_signal_heads(sh_idx);
    sg_id = in_sg_of_signal_heads(sh_idx);
    sg_obj = in_vissim.Net.SignalControllers.ItemByKey(sc_id).SGs.ItemByKey(sg_id);
    
    % Force COM control and set state to GREEN
    sg_obj.set('AttValue', 'ContrByCOM', true);
    sg_obj.set('AttValue', 'SigState', 'GREEN');
end
end
