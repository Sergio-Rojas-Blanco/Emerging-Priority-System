% DESCRIPTION: Computes the exact multi-link distance from the EV's current position
%              to a target signal head. Uses the SP chain for cumulative distance when
%              the target is on a different link.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function out_dist = Calculate_distance_to_sh(in_curr_link, in_curr_pos, in_target_sh_id, in_length_2d_links, in_pos_sh, in_link_sh, in_sp_current, in_sh_distances, in_sh_lookup, in_link_lookup)
% CALCULATE_DISTANCE_TO_SH - Computes exact multi-link distance to target signal head.
%
% Optimized version: uses precomputed lookup arrays instead of ismember.
%
% Inputs:
%   in_curr_link       - scalar, current link ID of the EV
%   in_curr_pos        - scalar, current position on link (m)
%   in_target_sh_id    - scalar, target signal head ID
%   in_length_2d_links - 1xN row vector, link lengths (m)
%   in_pos_sh          - 1xM row vector, signal head positions on link (m)
%   in_link_sh         - 1xM row vector, parent link of each signal head
%   in_sp_current      - 1xK row vector, signal head IDs in current Extended Green Wave (SP)
%   in_sh_distances    - 1xM cell, distances to adjacent signal heads (m)
%   in_sh_lookup       - Lookup vector: sh_id -> index (0 if not found)
%   in_link_lookup     - Lookup vector: link_id -> index (0 if not found)
%
% Output:
%   out_dist - scalar, exact metric distance to target signal head (m)

% Validate and resolve the target signal head index via O(1) lookup
if in_target_sh_id < 1 || in_target_sh_id > numel(in_sh_lookup)
    out_dist = inf; return;
end
sh_idx = in_sh_lookup(in_target_sh_id);
if sh_idx == 0, out_dist = inf; return; end

target_link_id = in_link_sh(sh_idx);
target_pos     = in_pos_sh(sh_idx);

if in_curr_link == target_link_id
    % Case A: Same link -> direct positional difference
    out_dist = target_pos - in_curr_pos;
else
    % Case B: Multi-link -> use the SP chain to accumulate distance
    [is_in_sp, target_pos_in_sp] = ismember(in_target_sh_id, in_sp_current);
    
    if ~is_in_sp || isempty(in_sp_current)
        % Fallback: target signal head not in SP chain
        if in_curr_link >= 1 && in_curr_link <= numel(in_link_lookup)
            curr_link_idx = in_link_lookup(in_curr_link);
            if curr_link_idx > 0
                % Distance from EV to link exit + target position on its link
                dist_to_exit = in_length_2d_links(curr_link_idx) - in_curr_pos;
                out_dist = dist_to_exit + target_pos;
            else
                out_dist = inf;
            end
        else
            out_dist = inf;
        end
    else
        % Compute accumulated distance along the SP chain
        total_dist = 0;
        
        % 1. Distance from EV to the first signal head in SP
        first_sh_id = in_sp_current(1);
        first_sh_idx = in_sh_lookup(first_sh_id);
        first_sh_link = in_link_sh(first_sh_idx);
        first_sh_pos = in_pos_sh(first_sh_idx);
        
        if first_sh_link == in_curr_link
            % First SP signal head is on the same link as the EV
            total_dist = first_sh_pos - in_curr_pos;
        else
            % Cross-link: distance to exit + first SP signal head position
            curr_link_idx = in_link_lookup(in_curr_link);
            if curr_link_idx > 0
                total_dist = in_length_2d_links(curr_link_idx) - in_curr_pos;
            end
            total_dist = total_dist + first_sh_pos;
        end
        
        % 2. Sum distances between consecutive signal heads in the SP chain
        for k = 1:(target_pos_in_sp - 1)
            curr_sh_id = in_sp_current(k);
            next_sh_id = in_sp_current(k + 1);
            
            curr_sh_idx = in_sh_lookup(curr_sh_id);
            next_sh_idx = in_sh_lookup(next_sh_id);
            
            if ~isempty(in_sh_distances) && numel(in_sh_distances) >= curr_sh_idx
                link_curr = in_link_sh(curr_sh_idx);
                link_next = in_link_sh(next_sh_idx);
                
                if link_curr == link_next
                    % Same link: absolute position difference
                    dist_step = abs(in_pos_sh(next_sh_idx) - in_pos_sh(curr_sh_idx));
                else
                    % Cross-link: exit distance + entry position
                    link_curr_idx = in_link_lookup(link_curr);
                    if link_curr_idx > 0
                        dist_step = (in_length_2d_links(link_curr_idx) - in_pos_sh(curr_sh_idx)) + in_pos_sh(next_sh_idx);
                    else
                        dist_step = 0;
                    end
                end
                total_dist = total_dist + dist_step;
            end
        end
        
        out_dist = total_dist;
    end
end

% Clamp to non-negative (EV may have slightly passed the target)
out_dist = max(0, out_dist);
end
