% DESCRIPTION: Detects whether the EV has entered a decision link (route confirmation
%              point after a bifurcation). Uses O(1) lookup for link index resolution.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_has_entered] = Has_entered_decision_link(in_curr_link, in_last_link, in_is_decision_link, in_link_lookup)
% HAS_ENTERED_DECISION_LINK Checks if the EV has crossed into a new decision link.
% A decision link is one where the vehicle confirms its trajectory after a bifurcation.
%
% INPUTS:
% - in_curr_link: VISSIM ID of the EV's current link.
% - in_last_link: VISSIM ID of the link in the previous step.
% - in_is_decision_link: Logical row vector indexed by position [1...N].
% - in_link_lookup: Lookup vector: link_id -> index (0 if not found).
%
% OUTPUTS:
% - out_has_entered: Boolean (true if just entered a link marked as decision).

out_has_entered = false;

% Only evaluate if there has been a physical link change (topological event)
if in_curr_link ~= in_last_link && in_curr_link >= 1 && in_curr_link <= numel(in_link_lookup)
    curr_idx = in_link_lookup(in_curr_link);
    if curr_idx > 0 && in_is_decision_link(curr_idx)
        out_has_entered = true;
    end
end
end
