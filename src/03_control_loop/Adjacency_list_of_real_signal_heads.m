% DESCRIPTION: Builds the signal-head-level adjacency list via BFS over
%              the link topology. For each signal head, finds all
%              immediately downstream signal heads and computes the
%              traversal distance to each one.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_adjacency_list_of_signal_heads, out_distances_to_adjacent_signal_heads] = Adjacency_list_of_real_signal_heads(in_no_of_links, in_no_of_signal_heads, in_link_of_signal_heads, in_signal_heads_of_links, in_from_link_of_links, in_to_link_of_links, in_is_conn_of_links, in_from_pos_of_links, in_to_pos_of_links, in_pos_of_signal_heads, in_length_2d_of_links, in_parallel_tolerance, in_conn_tolerance)

% Numerical tolerance to avoid floating-point equality issues in position comparisons
EPS = 1e-7;

% Count total links and signal heads in the network
num_links = numel(in_no_of_links);
num_sh = numel(in_no_of_signal_heads);

% Initialize output cell arrays with empty row vectors (one cell per signal head)
out_adjacency_list_of_signal_heads = repmat({zeros(1,0)}, 1, num_sh);
out_distances_to_adjacent_signal_heads = repmat({zeros(1,0)}, 1, num_sh);

% Early exit if the network has no signal heads or no links
if num_sh == 0 || num_links == 0, return; end

% Cast input ID vectors to double for consistent arithmetic
in_no_of_links = double(in_no_of_links);
in_no_of_signal_heads = double(in_no_of_signal_heads);
in_link_of_signal_heads = double(in_link_of_signal_heads);

% Map each signal head to its parent link index (position in in_no_of_links)
[~, link_of_sh_idx] = ismember(in_link_of_signal_heads, in_no_of_links);

% ---- Pre-sort signal heads by position within each link ----
% This enables efficient downstream lookup: the first signal head with
% position > entry point is the nearest downstream signal head on that link.
sorted_sh_ids = cell(1, num_links);
sorted_sh_pos = cell(1, num_links);
for li = 1:num_links
    % Get the signal head IDs assigned to this link
    curr_ids = double(in_signal_heads_of_links{li});
    if isempty(curr_ids), continue; end
    % Map signal head IDs to their indices in the global signal head array
    [~, s_idx] = ismember(curr_ids, in_no_of_signal_heads);
    valid = s_idx > 0;
    if ~any(valid), continue; end
    s_idx = s_idx(valid);
    curr_ids = curr_ids(valid);
    % Sort signal heads in ascending order of their position along the link
    [sorted_p, ord] = sort(in_pos_of_signal_heads(s_idx), 'ascend');
    sorted_sh_ids{li} = curr_ids(ord);
    sorted_sh_pos{li} = sorted_p;
end

% ---- Build connector lookup tables indexed by source link ----
% For each non-connector link, stores the from-position, to-position,
% and destination link index of every connector departing from it.
conn_fp = cell(1, num_links);        % connector from-position on parent link
conn_tp = cell(1, num_links);        % connector to-position on destination link
conn_dest_lidx = cell(1, num_links); % index of destination link

% Identify which links are connectors (IsConn == 1)
conn_mask = (in_is_conn_of_links == 1);

% Map connector from/to link IDs to their indices in in_no_of_links
[~, f_idx] = ismember(double(in_from_link_of_links(conn_mask)), in_no_of_links);
[~, t_idx] = ismember(double(in_to_link_of_links(conn_mask)), in_no_of_links);
fps = in_from_pos_of_links(conn_mask);
tps = in_to_pos_of_links(conn_mask);

% Group connectors by their source (from) link for O(1) retrieval during BFS
for i = 1:numel(f_idx)
    if f_idx(i) > 0 && t_idx(i) > 0
        conn_fp{f_idx(i)}(end+1) = fps(i);
        conn_tp{f_idx(i)}(end+1) = tps(i);
        conn_dest_lidx{f_idx(i)}(end+1) = t_idx(i);
    end
end

% ==== MAIN BFS LOOP: one search per signal head ====
for si = 1:num_sh
    % Resolve the parent link index of the current source signal head
    s_lidx = link_of_sh_idx(si);
    if s_lidx == 0, continue; end
    
    % Initialize BFS queue with the source signal head's link, position, and zero distance
    q_lidx = s_lidx;                        % queue: link indices to explore
    q_epos = in_pos_of_signal_heads(si);     % queue: entry position on each link
    q_dist = 0;                              % queue: accumulated distance from source
    
    % Pruning vectors: track the minimum entry position and distance per link
    % to avoid re-exploring a link from a worse (farther) entry point
    min_epos = inf(1, num_links);
    min_dist = inf(1, num_links);
    min_epos(s_lidx) = q_epos;
    min_dist(s_lidx) = 0;
    
    % Track the best (shortest) distance found to each downstream signal head
    best_dist_to_sh = inf(1, num_sh);
    % Accumulate IDs of discovered downstream adjacent signal heads
    adj_ids = [];

    % Process the BFS queue (grows dynamically as new links are enqueued)
    head = 1;
    while head <= numel(q_lidx)
        % Dequeue current link index, entry position, and accumulated distance
        L = q_lidx(head);
        ep = q_epos(head);
        da = q_dist(head);
        head = head + 1;

        % Retrieve the pre-sorted signal head positions and IDs on this link
        sp = sorted_sh_pos{L};
        sids = sorted_sh_ids{L};

        % block_p: position of the first downstream signal head on this link;
        % signal heads beyond this point "block" further connector traversal
        block_p = inf;

        % Find the first signal head strictly downstream of the entry position
        ds_idx = find(sp > ep + EPS, 1);
        
        % If a downstream signal head exists on this link, register it as adjacent
        if ~isempty(ds_idx)
            % The first downstream signal head defines the blocking position
            block_p = sp(ds_idx);

            % Include all co-located (parallel) signal heads within tolerance
            % (e.g., multi-lane signal heads at the same stop line)
            lim = block_p + in_parallel_tolerance + EPS;
            for k = ds_idx:numel(sp)
                % Stop once we exceed the parallel tolerance window
                if sp(k) > lim, break; end
                % Resolve global index of this signal head
                [~, s_idx] = ismember(sids(k), in_no_of_signal_heads);
                if s_idx > 0
                    % Compute distance: accumulated + gap from entry to this signal head
                    new_sh_dist = da + (sp(k) - ep);
                    % Update if this path is shorter than any previously found
                    if new_sh_dist < best_dist_to_sh(s_idx)
                        % First discovery: append to adjacency list
                        if isinf(best_dist_to_sh(s_idx)), adj_ids(end+1) = sids(k); end
                        best_dist_to_sh(s_idx) = new_sh_dist;
                    end
                end
            end
        end

        % ---- Expand BFS to downstream links via connectors ----
        % Case 1: Current link is itself a connector -> follow it to its destination link
        if in_is_conn_of_links(L) == 1
            dest_id = double(in_to_link_of_links(L));
            % Resolve destination link index
            [~, d_lidx] = ismember(dest_id, in_no_of_links);
            
            if d_lidx > 0
                % Entry position on the destination link = connector's to-position
                new_epos = in_to_pos_of_links(L);
                % Distance: accumulated + remaining length of connector from entry point
                new_d = da + max(0, in_length_2d_of_links(L) - ep);
                
                % Enqueue only if this path offers a better entry or shorter distance
                if new_epos < min_epos(d_lidx) - EPS || new_d < min_dist(d_lidx) - EPS
                    min_epos(d_lidx) = min(min_epos(d_lidx), new_epos);
                    min_dist(d_lidx) = min(min_dist(d_lidx), new_d);
                    q_lidx(end+1) = d_lidx; q_epos(end+1) = new_epos; q_dist(end+1) = new_d;
                end
            end
        % Case 2: Current link is a regular link -> check all connectors departing from it
        else
            % Retrieve all connector from-positions on this link
            c_fp = conn_fp{L};
            if isempty(c_fp), continue; end
            c_tp = conn_tp{L};
            c_dl = conn_dest_lidx{L};
            
            % Evaluate each outgoing connector
            for ci = 1:numel(c_fp)
                % can_pass: connector departs BEFORE the signal head shadow zone
                % (i.e., the connector branch-off is upstream of the blocking signal head)
                can_pass = (block_p - c_fp(ci)) > in_conn_tolerance + EPS;
                % is_fwd: connector branch-off is at or downstream of the current entry point
                is_fwd = (c_fp(ci) >= ep - in_conn_tolerance - EPS);
                
                % Only traverse if the connector is both reachable and not blocked
                if can_pass && is_fwd
                    d_lidx = c_dl(ci);
                    % Entry position on destination link = connector's to-position
                    new_epos = c_tp(ci);
                    % Distance: accumulated + gap from entry to connector branch-off
                    new_d = da + max(0, c_fp(ci) - ep);
                    
                    % Enqueue only if this path offers a better entry or shorter distance
                    if new_epos < min_epos(d_lidx) - EPS || new_d < min_dist(d_lidx) - EPS
                        min_epos(d_lidx) = min(min_epos(d_lidx), new_epos);
                        min_dist(d_lidx) = min(min_dist(d_lidx), new_d);
                        q_lidx(end+1) = d_lidx; q_epos(end+1) = new_epos; q_dist(end+1) = new_d;
                    end
                end
            end
        end
    end
    
    % ---- Store results for this source signal head ----
    if ~isempty(adj_ids)
        out_adjacency_list_of_signal_heads{si} = adj_ids;
        % Collect the shortest distance to each discovered adjacent signal head
        dists = zeros(1, numel(adj_ids));
        for k = 1:numel(adj_ids)
            [~, s_idx] = ismember(adj_ids(k), in_no_of_signal_heads);
            dists(k) = best_dist_to_sh(s_idx);
        end
        out_distances_to_adjacent_signal_heads{si} = dists;
    end
end
end
