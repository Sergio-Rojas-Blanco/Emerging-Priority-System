% DESCRIPTION: Generates virtual (fictitious) signal heads at each detected geometric
%              crossing point between non-adjacent links. Each crossing produces one
%              virtual signal head shared by both intersecting links, enabling
%              downstream conflict detection via BFS.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_no_of_signal_heads_by_crossing, out_pos_of_signal_heads_by_crossing, out_links_of_signal_heads_by_crossing, out_signal_heads_of_links_by_crossing] = Add_signal_heads_in_crossings_links(in_id_crossing_links, in_dist_crossing_links, in_no_of_links, in_no_of_signal_heads, in_from_link_of_links)

num_links = numel(in_no_of_links);

% Initialize per-link cell array for crossing virtual signal head IDs
out_signal_heads_of_links_by_crossing = cell(1, num_links);

% ---- Collect unique crossing pairs ----
% Each geometric crossing is recorded symmetrically in id_crossing_links.
% To avoid duplicates, only process pair (i, j) where i < j.
unique_pairs = {};
for i = 1:num_links
    cross_ids = double(in_id_crossing_links{i});
    cross_dists = in_dist_crossing_links{i};
    if isempty(cross_ids), continue; end

    % Map crossing link IDs to their indices in the link array
    [~, cross_idx] = ismember(cross_ids, double(in_no_of_links));

    for j = 1:numel(cross_ids)
        c_idx = cross_idx(j);
        
        % Skip unresolved IDs; enforce i < c_idx to process each pair once
        if c_idx == 0 || i >= c_idx
            continue;
        end

        % Skip pairs that share the same parent (from) link:
        % these are branches from the same junction, not true crossings
        if nargin >= 5 && ~isempty(in_from_link_of_links)
            f1 = in_from_link_of_links(i); 
            f2 = in_from_link_of_links(c_idx);
            if f1 > 0 && f2 > 0 && f1 == f2, continue; end
        end

        % Distance along link i to the crossing point
        dist1 = cross_dists(j);
        % Find the reciprocal distance along link j to the same crossing point
        dist2 = NaN;

        % Look up the reciprocal entry: link j's crossing list should contain link i
        recip_mask = (in_id_crossing_links{c_idx} == in_no_of_links(i));
        if any(recip_mask)
            d2_vals = in_dist_crossing_links{c_idx}(recip_mask);
            dist2 = d2_vals(1); 
        end

        % Store the unique pair: [link_i_idx, link_j_idx, dist_on_i, dist_on_j]
        unique_pairs{end+1} = [i, c_idx, dist1, dist2]; %#ok<AGROW>
    end
end

num_pairs = numel(unique_pairs);

% Early exit if no crossing pairs were detected
if num_pairs == 0
    out_no_of_signal_heads_by_crossing = [];
    out_pos_of_signal_heads_by_crossing = {};
    out_links_of_signal_heads_by_crossing = {};
    return;
end

% ---- Assign unique IDs to crossing virtual signal heads ----
% IDs start after the maximum existing real signal head ID
next_id = 1;
if ~isempty(in_no_of_signal_heads)
    next_id = max(in_no_of_signal_heads) + 1;
end
out_no_of_signal_heads_by_crossing = next_id : (next_id + num_pairs - 1);

% Each crossing signal head has two positions (one per link) and two parent links
out_pos_of_signal_heads_by_crossing = cell(1, num_pairs);
out_links_of_signal_heads_by_crossing = cell(1, num_pairs);

% Populate crossing signal head attributes and register them in per-link arrays
for i = 1:num_pairs
    p = unique_pairs{i};
    idx1 = p(1); idx2 = p(2);

    % Parent link IDs for this crossing signal head
    out_links_of_signal_heads_by_crossing{i} = [in_no_of_links(idx1), in_no_of_links(idx2)];
    % Positions along each parent link where the crossing occurs
    out_pos_of_signal_heads_by_crossing{i} = [p(3), p(4)];

    % Register this virtual signal head in both parent links' per-link arrays
    s_id = out_no_of_signal_heads_by_crossing(i);
    out_signal_heads_of_links_by_crossing{idx1}(end+1) = s_id;
    out_signal_heads_of_links_by_crossing{idx2}(end+1) = s_id;
end
end
