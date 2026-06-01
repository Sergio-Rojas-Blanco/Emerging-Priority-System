% DESCRIPTION: Dynamic control loop module. Executes VISSIM COM interface, real-time telemetry, and Extended Green Wave (SP) / Red Line (SC) priority algorithms.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_sc_new] = Calculate_sc(in_signal_heads_conflict_list, in_no_of_signal_heads, in_sp_new)
% CALCULATE_SC - Computes the Red Line (SC): signal heads in conflict with the Extended Green Wave
%
% For each prioritized signal head (SP), retrieves all its topological conflicts
% from in_signal_heads_conflict_list. Excludes those already in SP (a prioritized
% signal head is never set to red for itself).
%
% CURRENT SIMPLIFICATION (v1):
%   ALL signal heads in topological conflict with the SP are included,
%   regardless of their distance to the EV. This means that distant signal heads
%   will be set to red even if their activation does not benefit the EV,
%   unnecessarily penalizing civilian traffic.
%
% FUTURE RESEARCH LINES:
%   - Filter SC by distance to the EV (dynamic conflict horizon).
%   - Apply kinematic filter by t_clearance (clearance time).
%   - Consider VisibilityFactor and PedestrianRiskFactor.
%   - Introduce transition time between SC (red) and SP (green) activation
%     to respect the physical clearance time of the intersection.
%   - Prioritize near SC over distant SC to minimize impact on civilian traffic.
%
% Inputs:
%   in_signal_heads_conflict_list - 1xM cell, conflicting signal head IDs per SH
%   in_no_of_signal_heads         - 1xM row vector, all signal head IDs
%   in_sp_new                     - 1xK row vector, prioritized signal head IDs (SP)
%
% Output:
%   out_sc_new - 1xJ row vector, signal head IDs to be set to RED

out_sc_new = [];
if isempty(in_sp_new), return; end

max_sh_id = max(in_no_of_signal_heads);
sh_lookup = zeros(1, max_sh_id);
sh_lookup(in_no_of_signal_heads) = 1:numel(in_no_of_signal_heads);

% Mark prioritized signal heads
sp_mask = false(1, max_sh_id);
valid_sp = in_sp_new(in_sp_new >= 1 & in_sp_new <= max_sh_id);
valid_sp = valid_sp(sh_lookup(valid_sp) > 0);
sp_mask(valid_sp) = true;

% Find conflicting signal heads
sc_mask = false(1, max_sh_id);
for i = 1:numel(valid_sp)
    sp_idx = sh_lookup(valid_sp(i));
    conflict_ids = in_signal_heads_conflict_list{sp_idx};
    valid_c = conflict_ids(conflict_ids >= 1 & conflict_ids <= max_sh_id);
    sc_mask(valid_c) = true;
end

% Exclude SPs from the conflict list
sc_mask(sp_mask) = false;
out_sc_new = find(sc_mask);
out_sc_new = out_sc_new(sh_lookup(out_sc_new) > 0);

end
