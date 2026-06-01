% DESCRIPTION: Computes topological conflict matrices (crossing and convergence) between signal head groups.
%              Detects geometric intersections, injects virtual signal heads at endpoints and crossing
%              points, performs BFS over the extended networks, and merges endpoint and crossing
%              conflicts into a unified conflict list with distances and ETAs.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_conflict_list_of_signal_heads, out_distance_to_conflicting_signal_heads_crossing, out_eta_to_conflicting_signal_heads_crossing] = Conflict_list_of_signal_heads(in_adjacency_list_of_links, in_no_of_links, in_link_of_signal_heads, in_no_of_signal_heads, in_pos_of_signal_heads, in_signal_heads_of_links, in_length_2D_of_links, in_coords_of_links, in_from_link_of_links, in_to_link_of_links, in_is_conn_of_links, in_from_pos_of_links, in_to_pos_of_links, in_connection_point_of_adjacent, in_conn_tolerance, in_parallel_tolerance, in_mean_speed_kmh)

% Step 1: Detect geometric intersections between link bounding boxes.
% Returns pairs of crossing links and their intersection distances along
% each link's polyline. This identifies where two non-adjacent links
% physically overlap in the 2D plane (potential crossing conflicts).
[id_crossing_links, dist_crossing_links] = Crossing_links_bounding_boxes(in_coords_of_links, in_adjacency_list_of_links, in_no_of_links, in_from_link_of_links, in_to_link_of_links, in_is_conn_of_links, in_from_pos_of_links, in_conn_tolerance);

% Step 2: Generate virtual (fictitious) signal heads at two types of locations:
%   a) Endpoint signal heads: placed at the downstream end of each link,
%      representing the point where traffic exits a link via a connector.
[no_sh_end, pos_sh_end, link_sh_end, sh_of_links_end] = Add_signal_heads_in_endpoints_links(in_adjacency_list_of_links, in_connection_point_of_adjacent, in_no_of_links, in_length_2D_of_links, in_no_of_signal_heads, in_conn_tolerance, in_is_conn_of_links);

%   b) Crossing signal heads: placed at each detected geometric intersection
%      point, representing the conflict zone where two link trajectories cross.
[no_sh_cross, pos_sh_cross, link_sh_cross, sh_of_links_cross] = Add_signal_heads_in_crossings_links(id_crossing_links, dist_crossing_links, in_no_of_links, in_no_of_signal_heads, in_from_link_of_links);

% Step 3: Unify real and virtual signal head networks into extended sets.
% Two separate extended networks are built:
%   - ext_end:   real signal heads + endpoint virtual signal heads
%                (used to detect convergence conflicts at link endpoints)
%   - ext_cross: real signal heads + crossing virtual signal heads
%                (used to detect crossing conflicts at intersection zones)
[no_sh_ext_end, pos_sh_ext_end, link_sh_ext_end, sh_of_links_ext_end] = Unify_ficticial_signal_heads(in_no_of_signal_heads, in_pos_of_signal_heads, in_link_of_signal_heads, in_signal_heads_of_links, no_sh_end, pos_sh_end, link_sh_end, sh_of_links_end, no_sh_cross, pos_sh_cross, link_sh_cross, sh_of_links_cross, in_no_of_links, true, false);
[no_sh_ext_cross, pos_sh_ext_cross, link_sh_ext_cross, sh_of_links_ext_cross] = Unify_ficticial_signal_heads(in_no_of_signal_heads, in_pos_of_signal_heads, in_link_of_signal_heads, in_signal_heads_of_links, no_sh_end, pos_sh_end, link_sh_end, sh_of_links_end, no_sh_cross, pos_sh_cross, link_sh_cross, sh_of_links_cross, in_no_of_links, false, true);

% Step 4: Run BFS-based adjacency on each extended signal head network.
% This produces adjacency lists over the augmented graphs, enabling
% conflict detection by tracing which real signal heads share a common
% downstream virtual signal head.
%   - adj_list_end:   adjacency over endpoint-extended network (for convergence)
%   - adj_list_cross: adjacency over crossing-extended network (for crossing conflicts)
[adj_list_end, ~] = Adjacency_list_of_fictitious_signal_heads(no_sh_ext_end, link_sh_ext_end, sh_of_links_ext_end, in_no_of_links, in_from_link_of_links, in_to_link_of_links, in_is_conn_of_links, in_from_pos_of_links, in_to_pos_of_links, pos_sh_ext_end, in_length_2D_of_links, in_parallel_tolerance, in_conn_tolerance);
[adj_list_cross, dist_list_cross] = Adjacency_list_of_fictitious_signal_heads(no_sh_ext_cross, link_sh_ext_cross, sh_of_links_ext_cross, in_no_of_links, in_from_link_of_links, in_to_link_of_links, in_is_conn_of_links, in_from_pos_of_links, in_to_pos_of_links, pos_sh_ext_cross, in_length_2D_of_links, in_parallel_tolerance, in_conn_tolerance);

% Step 5: Compute conflict lists from each extended adjacency.
% Endpoint conflicts (convergence): two real signal heads whose downstream
% paths converge at the same endpoint virtual signal head.
[conflict_list_end] = Conflict_list_of_signal_heads_endpoints(adj_list_end, no_sh_ext_end, in_no_of_signal_heads, link_sh_ext_end, pos_sh_ext_end, in_parallel_tolerance);

% Crossing conflicts: two real signal heads whose downstream paths pass
% through the same crossing virtual signal head. Also returns the traversal
% distance and estimated time of arrival (ETA) to each conflict point.
[conflict_list_cross, out_distance_to_conflicting_signal_heads_crossing, out_eta_to_conflicting_signal_heads_crossing] = Conflict_list_of_signal_heads_crossing(adj_list_cross, dist_list_cross, in_no_of_signal_heads, no_sh_ext_cross, link_sh_ext_cross, in_mean_speed_kmh);

% Step 6: Merge endpoint and crossing conflict lists per signal head.
% For each signal head, the final conflict set is the union of both
% conflict types, preserving discovery order via 'stable' uniqueness.
out_conflict_list_of_signal_heads = cellfun(@(a,b) unique([a, b], 'stable'), conflict_list_end, conflict_list_cross, 'UniformOutput', false);

end
