% DESCRIPTION: Computes the physical infrastructure resistance matrix. Integrates
%              turn penalties (angular deflection), lane maneuverability, and
%              acceleration potential based on link length.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_infrastructure_factor] = Calculate_infrastructure_factor(in_no_of_links, in_num_lanes_of_links, in_length_2D_of_links, in_coords_of_links, in_links_adjacency_list)
% CALCULATE_INFRASTRUCTURE_FACTOR Computes the physical infrastructure resistance.
% Integrates turn penalties (geometry), lane maneuverability, and acceleration
% potential based on link length.
% NOTE: Future versions may incorporate signal head density per link, lateral
% friction from parking, or length/curvature ratio to adjust the dynamic
% permeability of the emergency flow.
%
% INPUTS:
% - in_no_of_links: Row vector with VISSIM link IDs.
% - in_num_lanes_of_links: Row vector with the number of lanes per link [link_idx].
% - in_length_2D_of_links: Row vector with the total length of each link [link_idx].
% - in_coords_of_links: Row cell with [X, Y, Rad] matrices per link [link_idx].
% - in_links_adjacency_list: Row cell with adjacent link IDs per link [link_idx].
%
% OUTPUTS:
% - out_infrastructure_factor: [n_links x n_links] matrix with transition cost in seconds.

num_links = length(in_no_of_links);
out_infrastructure_factor = zeros(num_links, num_links);

% Iterate over each link as the origin of a potential transition
for i = 1:num_links
    adj_ids = in_links_adjacency_list{i};
    if isempty(adj_ids), continue; end
    
    num_lanes = in_num_lanes_of_links(i);
    link_len = in_length_2D_of_links(i);
    coords_origin = in_coords_of_links{i};
    % Skip links with fewer than 2 polyline points (cannot compute direction)
    if size(coords_origin, 1) < 2, continue; end
    
    % 1. Lane factor: penalty for narrowing.
    % Fewer lanes make it harder for civilian traffic to yield.
    lane_penalty = 1.2 / max(1, num_lanes); 

    % 2. Acceleration/length factor:
    % Short links (<50m) prevent the EV from reaching V_desired, adding inertia.
    acceleration_penalty = max(0, (50 - link_len) / 50);

    % Evaluate each downstream neighbour
    for j = 1:length(adj_ids)
        [~, dest_idx] = ismember(adj_ids(j), in_no_of_links);
        if dest_idx == 0, continue; end % Skip if adjacent link not in list
        
        coords_dest = in_coords_of_links{dest_idx};
        if size(coords_dest, 1) < 2, continue; end
        
        % 3. Angular deflection (turn) calculation
        % Compute exit direction of origin link and entry direction of destination link
        vec_origin = coords_origin(end, 1:2) - coords_origin(end-1, 1:2);
        vec_dest = coords_dest(2, 1:2) - coords_dest(1, 1:2);
        
        % Normalize to unit vectors and compute the angle between them
        unit_orig = vec_origin / norm(vec_origin);
        unit_dest = vec_dest / norm(vec_dest);
        dot_prod = max(-1, min(1, dot(unit_orig, unit_dest)));
        angle_rad = acos(dot_prod);
        
        % Total cost = Turn + Lanes + Inertia from short link length
        out_infrastructure_factor(i, dest_idx) = (angle_rad * 2.0) + lane_penalty + acceleration_penalty;
    end
end
end
