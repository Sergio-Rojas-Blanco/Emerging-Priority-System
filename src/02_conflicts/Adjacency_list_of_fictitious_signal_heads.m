% DESCRIPTION: BFS-based adjacency list builder for extended (real + fictitious) signal
%              head networks. Identical algorithm to Adjacency_list_of_real_signal_heads
%              but operates over the augmented graph that includes virtual signal heads.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_adj, out_dist] = Adjacency_list_of_fictitious_signal_heads(in_no_ext, in_link_ext, in_sh_of_links_ext, in_no_of_links, in_from_link, in_to_link, in_is_conn, in_from_pos, in_to_pos, in_pos_ext, in_length_2d, in_parallel_tol, in_conn_tol)

% Numerical tolerance for floating-point position comparisons
EPS = 1e-7;
num_links = numel(in_no_of_links);
num_sh = numel(in_no_ext);

% Initialize output cell arrays with empty row vectors
out_adj = repmat({zeros(1,0)}, 1, num_sh);
out_dist = repmat({zeros(1,0)}, 1, num_sh);
if num_sh == 0 || num_links == 0, return; end

% Map each extended signal head to its parent link index
[~, link_of_sh_idx] = ismember(double(in_link_ext), double(in_no_of_links));

% ---- Pre-sort extended signal heads by position within each link ----
sorted_sh_ids = cell(1, num_links);
sorted_sh_pos = cell(1, num_links);
for li = 1:num_links
    curr_ids = double(in_sh_of_links_ext{li});
    if isempty(curr_ids), continue; end
    [~, s_idx] = ismember(curr_ids, double(in_no_ext));
    valid = s_idx > 0;
    if ~any(valid), continue; end
    s_idx = s_idx(valid);
    % Sort by ascending position along the link
    [sorted_p, ord] = sort(in_pos_ext(s_idx), 'ascend');
    sorted_sh_ids{li} = curr_ids(valid(ord));
    sorted_sh_pos{li} = sorted_p;
end

% ---- Build connector lookup tables indexed by source link ----
conn_fp = cell(1, num_links); conn_tp = cell(1, num_links); conn_dl = cell(1, num_links);
conn_mask = (in_is_conn == 1);
[~, f_idx] = ismember(double(in_from_link(conn_mask)), double(in_no_of_links));
[~, t_idx] = ismember(double(in_to_link(conn_mask)), double(in_no_of_links));
fps = in_from_pos(conn_mask); tps = in_to_pos(conn_mask);

% Group connectors by source link
for i = 1:numel(f_idx)
    if f_idx(i) > 0 && t_idx(i) > 0
        conn_fp{f_idx(i)}(end+1) = fps(i);
        conn_tp{f_idx(i)}(end+1) = tps(i);
        conn_dl{f_idx(i)}(end+1) = t_idx(i);
    end
end

% ==== BFS loop: one search per extended signal head ====
for si = 1:num_sh
    s_lidx = link_of_sh_idx(si);
    if s_lidx == 0, continue; end
    
    % Initialize BFS queue with source signal head's link, position, distance
    q_lidx = s_lidx; q_epos = in_pos_ext(si); q_dist = 0;
    % Pruning: track minimum entry position and distance per link
    min_epos = inf(1, num_links); min_dist = inf(1, num_links);
    min_epos(s_lidx) = q_epos; min_dist(s_lidx) = 0;
    
    % Track best distance to each downstream signal head
    best_dist = inf(1, num_sh); adj_ids = []; head = 1;
    
    % Process the BFS queue
    while head <= numel(q_lidx)
        % Dequeue current link, entry position, accumulated distance
        L = q_lidx(head); ep = q_epos(head); da = q_dist(head); head = head + 1;

        % Retrieve pre-sorted signal heads on this link
        sp = sorted_sh_pos{L}; sids = sorted_sh_ids{L};
        block_p = inf;
        % Find first downstream signal head strictly after entry position
        ds_idx = find(sp > ep + EPS, 1);
        
        % If downstream signal heads exist, register them as adjacent
        if ~isempty(ds_idx)
            block_p = sp(ds_idx);
            lim = block_p + in_parallel_tol + EPS;
            % Include all co-located (parallel) signal heads within tolerance
            for k = ds_idx:numel(sp)
                if sp(k) > lim, break; end
                [~, s_idx] = ismember(sids(k), in_no_ext);
                if s_idx > 0 && da + (sp(k) - ep) < best_dist(s_idx)
                    % First discovery: append to adjacency list
                    if isinf(best_dist(s_idx)), adj_ids(end+1) = sids(k); end %#ok<AGROW> 
                    best_dist(s_idx) = da + (sp(k) - ep);
                end
            end
        end

        % ---- Expand BFS to downstream links ----
        if in_is_conn(L) == 1
            % Current link is a connector: follow to destination link
            [~, d_lidx] = ismember(double(in_to_link(L)), double(in_no_of_links));
            if d_lidx > 0
                new_epos = in_to_pos(L); new_d = da + max(0, in_length_2d(L) - ep);
                if new_epos < min_epos(d_lidx) - EPS || new_d < min_dist(d_lidx) - EPS
                    min_epos(d_lidx) = min(min_epos(d_lidx), new_epos); min_dist(d_lidx) = min(min_dist(d_lidx), new_d);
                    q_lidx(end+1) = d_lidx; q_epos(end+1) = new_epos; q_dist(end+1) = new_d; %#ok<AGROW> 
                end
            end
        else
            % Current link is regular: check all outgoing connectors
            c_fp = conn_fp{L}; if isempty(c_fp), continue; end
            for ci = 1:numel(c_fp)
                % can_pass: connector is upstream of the blocking signal head
                % is_fwd: connector is at or downstream of the entry point
                if (block_p - c_fp(ci)) > in_conn_tol + EPS && (c_fp(ci) >= ep - in_conn_tol - EPS)
                    d_lidx = conn_dl{L}(ci); new_epos = conn_tp{L}(ci); new_d = da + max(0, c_fp(ci) - ep);
                    if new_epos < min_epos(d_lidx) - EPS || new_d < min_dist(d_lidx) - EPS
                        min_epos(d_lidx) = min(min_epos(d_lidx), new_epos); min_dist(d_lidx) = min(min_dist(d_lidx), new_d);
                        q_lidx(end+1) = d_lidx; q_epos(end+1) = new_epos; q_dist(end+1) = new_d; %#ok<AGROW> 
                    end
                end
            end
        end
    end
    % Store adjacency results for this source signal head
    if ~isempty(adj_ids)
        out_adj{si} = adj_ids;
        [~, s_idxs] = ismember(adj_ids, in_no_ext);
        out_dist{si} = best_dist(s_idxs);
    end
end
end
