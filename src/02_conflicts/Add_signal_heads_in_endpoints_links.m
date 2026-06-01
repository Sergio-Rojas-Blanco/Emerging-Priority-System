% DESCRIPTION: Generates virtual (fictitious) signal heads at the downstream endpoint
%              of each terminal link. A terminal link is one whose downstream end is
%              not connected to any outgoing connector within tolerance.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_no_of_signal_heads_by_endpoints, out_pos_of_signal_heads_by_endpoints, out_link_of_signal_heads_by_endpoints, out_signal_heads_of_links_by_endpoints] = Add_signal_heads_in_endpoints_links(in_adjacency_list_of_links, in_connection_point_of_adjacency_list, in_no_of_links, in_length_2D_of_links, in_no_of_signal_heads, in_conn_tolerance, in_is_conn_of_links)

num_links = numel(in_no_of_links);

% Initialize output: one cell per link to store assigned endpoint signal head IDs
out_signal_heads_of_links_by_endpoints = cell(1, num_links);
for i = 1:num_links, out_signal_heads_of_links_by_endpoints{i} = zeros(1,0); end

% ---- Identify terminal links ----
% A link is terminal if it has no downstream neighbours, or if none of its
% connection points lie at the link's downstream end (within tolerance).
is_terminal = false(1, num_links);
for i = 1:num_links
    adj_links = in_adjacency_list_of_links{i};
    conn_pts = in_connection_point_of_adjacency_list{i};
    
    % Case 1: no downstream neighbours at all
    if isempty(adj_links)
        is_terminal(i) = true;
    % Case 2: neighbours exist but none depart from the link's downstream end
    elseif isempty(conn_pts) || ~any(abs(conn_pts - in_length_2D_of_links(i)) <= in_conn_tolerance)
        is_terminal(i) = true;
    end
end

% Exclude connectors from terminal classification: connectors are intermediate
% link elements and should never receive endpoint virtual signal heads
if nargin >= 7 && ~isempty(in_is_conn_of_links)
    is_terminal(in_is_conn_of_links == 1) = false;
end

% Collect indices of all terminal links
term_idx = find(is_terminal);
num_term = numel(term_idx);

% Early exit if no terminal links exist in the network
if num_term == 0
    out_no_of_signal_heads_by_endpoints = [];
    out_pos_of_signal_heads_by_endpoints = [];
    out_link_of_signal_heads_by_endpoints = [];
    return;
end

% ---- Assign unique IDs to endpoint virtual signal heads ----
% IDs start after the maximum existing real signal head ID to avoid collisions
next_id = 1;
if ~isempty(in_no_of_signal_heads)
    next_id = max(in_no_of_signal_heads) + 1;
end

% Generate consecutive IDs for each terminal link
out_no_of_signal_heads_by_endpoints = next_id : (next_id + num_term - 1);

% Position each virtual signal head at the downstream end of its terminal link
out_pos_of_signal_heads_by_endpoints = in_length_2D_of_links(term_idx);

% Record the parent link ID for each virtual signal head
out_link_of_signal_heads_by_endpoints = in_no_of_links(term_idx);

% Map each virtual signal head back to its parent link's per-link cell array
for k = 1:num_term
    idx = term_idx(k);
    out_signal_heads_of_links_by_endpoints{idx} = out_no_of_signal_heads_by_endpoints(k);
end
end
