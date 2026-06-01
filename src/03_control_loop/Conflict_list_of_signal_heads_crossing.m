% DESCRIPTION: Detects crossing conflicts between real signal heads by identifying
%              pairs that share a common downstream virtual crossing signal head.
%              Returns conflict lists, traversal distances, and ETAs per signal head.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_conflicts, out_dist, out_eta] = Conflict_list_of_signal_heads_crossing(in_adj_cross, in_dist_cross, in_no_real, in_no_ext, in_link_ext, in_speed_kmh)

num_real = numel(in_no_real);
% Convert mean traffic speed from km/h to m/s for ETA computation
mean_speed_mps = in_speed_kmh / 3.6;

% Initialize output cell arrays: one cell per real signal head
out_conflicts = repmat({zeros(1,0)}, 1, num_real);
out_dist = repmat({zeros(1,0)}, 1, num_real);
out_eta = repmat({zeros(1,0)}, 1, num_real);

% Map real signal head IDs to their indices in the extended array
[~, real_idx_map] = ismember(in_no_real, in_no_ext);

% ---- Build inverted index: crossing node -> list of (source, distance) ----
% For each real signal head, trace its adjacency to find downstream virtual
% crossing nodes (IDs greater than max real ID). Record which real signal
% heads reach each crossing node and at what distance.
cross_to_sources = containers.Map('KeyType', 'double', 'ValueType', 'any');
for r = 1:num_real
    e_idx = real_idx_map(r);
    if e_idx == 0, continue; end
    targets = in_adj_cross{e_idx};
    dists = in_dist_cross{e_idx};
    
    % Iterate over downstream neighbours in the extended adjacency
    for t = 1:numel(targets)
        tid = targets(t);
        % Only consider virtual crossing nodes (fictitious IDs above the real range)
        if tid > max(in_no_real)
            if isKey(cross_to_sources, tid)
                mat = cross_to_sources(tid);
                cross_to_sources(tid) = [mat; [in_no_real(r), dists(t)]];
            else
                cross_to_sources(tid) = [in_no_real(r), dists(t)];
            end
        end
    end
end

% ---- Resolve conflicts: two real signal heads sharing a crossing node ----
targets = keys(cross_to_sources);
for t = 1:numel(targets)
    mat = cross_to_sources(targets{t});
    % A crossing conflict requires at least 2 distinct sources
    if size(mat, 1) < 2, continue; end
    
    sources = mat(:, 1)';
    dists = mat(:, 2)';
    % Resolve parent links for each source signal head
    [~, s_idxs] = ismember(sources, in_no_ext);
    links = in_link_ext(s_idxs);
    
    % Pairwise conflict check among all sources of this crossing node
    for i = 1:numel(sources)
        [~, r_idx] = ismember(sources(i), in_no_real);
        for j = i+1:numel(sources)
            % Crossing conflict: sources must come from different links
            % (same-link sources are parallel, not crossing)
            if links(i) ~= links(j)
                % Register conflict symmetrically for both signal heads
                out_conflicts{r_idx}(end+1) = sources(j);
                out_dist{r_idx}(end+1) = dists(j);
                out_eta{r_idx}(end+1) = dists(j) / mean_speed_mps;
                
                [~, r_jdx] = ismember(sources(j), in_no_real);
                out_conflicts{r_jdx}(end+1) = sources(i);
                out_dist{r_jdx}(end+1) = dists(i);
                out_eta{r_jdx}(end+1) = dists(i) / mean_speed_mps;
            end
        end
    end
end

% ---- Deduplicate conflicts per signal head ----
for i = 1:num_real
    if ~isempty(out_conflicts{i})
        [u_conf, ia] = unique(out_conflicts{i}, 'stable');
        out_conflicts{i} = u_conf;
        out_dist{i} = out_dist{i}(ia);
        out_eta{i} = out_eta{i}(ia);
    end
end
end
