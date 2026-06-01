% DESCRIPTION: Detects the edge event when the EV physically passes a prioritized
%              signal head. Fires only on the step where the EV's position crosses
%              the signal head position (falling-edge detection).
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function out_passed = Has_passed_signal_head(in_curr_ev_link, in_curr_ev_pos, in_last_ev_pos, in_sp_current, in_pos_sh, in_link_sh, in_sh_lookup)
% HAS_PASSED_SIGNAL_HEAD - Edge detection: fires only on the step where the EV
% surpasses the position of a prioritized signal head.
% in_last_ev_pos: EV position in the previous step (same link; Inf if link changed).
% in_sh_lookup: Lookup vector: sh_id -> index (0 if not found).

out_passed = false;

if isempty(in_sp_current), return; end

% Resolve indices of prioritized signal heads via O(1) lookup per element
n_sp = numel(in_sp_current);
valid_indices = zeros(1, n_sp);
n_valid = 0;
max_sh = numel(in_sh_lookup);
for i = 1:n_sp
    sh_id = in_sp_current(i);
    if sh_id >= 1 && sh_id <= max_sh
        idx = in_sh_lookup(sh_id);
        if idx > 0
            n_valid = n_valid + 1;
            valid_indices(n_valid) = idx;
        end
    end
end
if n_valid == 0, return; end
valid_indices = valid_indices(1:n_valid);

% Filter by link and detect crossing edge (before <= sh_pos, now > sh_pos)
relevant_links = in_link_sh(valid_indices);
is_in_same_link = (relevant_links == in_curr_ev_link);

if any(is_in_same_link)
    relevant_positions = in_pos_sh(valid_indices(is_in_same_link));
    % Edge detection: EV was at or before the signal head, now past it
    if any(in_last_ev_pos <= relevant_positions & in_curr_ev_pos > relevant_positions)
        out_passed = true;
    end
end
end
