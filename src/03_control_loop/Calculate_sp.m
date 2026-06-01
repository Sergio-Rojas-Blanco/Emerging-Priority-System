% DESCRIPTION: Computes the Extended Green Wave (SP) via BFS over the signal head
%              adjacency graph. Expands from the EV's current position with a dynamic
%              horizon: cumulative ETA <= (link clearance time + safety margin).
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_sp_new] = Calculate_sp(in_signal_heads_adjacency_list, in_signal_heads_distances, in_no_of_links, in_no_of_signal_heads, in_signal_heads_of_links, in_pos_of_signal_heads, in_length_2d_of_links, in_conn_tolerance, in_curr_ev_link, in_curr_ev_pos, in_clearance_time_of_links, in_safety_margin, in_mean_speed_kmh, in_queue_capacity, in_queue_growth, in_sp_capacity)

% Numerical tolerance for floating-point position comparisons
EPS = 1e-9;
out_sp_new = [];

% Early exit if no valid EV link
if isempty(in_curr_ev_link) || in_curr_ev_link == 0, return; end

% Convert EV desired speed from km/h to m/s
speed_mps = in_mean_speed_kmh / 3.6;
if speed_mps <= 0, return; end

% ---- Build O(1) lookup arrays ----
num_sh = numel(in_no_of_signal_heads);
max_sh_id = max(in_no_of_signal_heads);
sh_lookup = zeros(1, max_sh_id);
sh_lookup(in_no_of_signal_heads) = 1:num_sh;

max_link_id = max(in_no_of_links);
link_lookup = zeros(1, max_link_id);
link_lookup(in_no_of_links) = 1:numel(in_no_of_links);

% Map each signal head ID to its parent link index
sh_to_link_idx = zeros(1, max_sh_id);
for li = 1:numel(in_no_of_links)
    shs = in_signal_heads_of_links{li};
    for s = 1:numel(shs)
        if shs(s) >= 1 && shs(s) <= max_sh_id
            sh_to_link_idx(shs(s)) = li;
        end
    end
end

% Resolve the EV's current link index
curr_link_idx = link_lookup(in_curr_ev_link);
if curr_link_idx == 0, return; end

% ---- Initialize BFS queue (pre-allocated for performance) ----
queue_ids   = zeros(1, in_queue_capacity);
queue_dists = zeros(1, in_queue_capacity);
q_tail = 0;

% ---- SEED PHASE 1: signal heads on the EV's current link, downstream of EV ----
sh_in_curr_link = in_signal_heads_of_links{curr_link_idx};

for i = 1:numel(sh_in_curr_link)
    sh_id = sh_in_curr_link(i);
    if sh_id < 1 || sh_id > max_sh_id, continue; end
    sh_idx = sh_lookup(sh_id);
    if sh_idx == 0, continue; end
    sh_pos = in_pos_of_signal_heads(sh_idx);
    
    % Only consider signal heads strictly downstream of the EV's current position
    if sh_pos <= in_curr_ev_pos + EPS, continue; end
    
    % Compute distance and ETA from EV to this signal head
    d = sh_pos - in_curr_ev_pos;
    eta = d / speed_mps;
    t_despeje = in_clearance_time_of_links(curr_link_idx);
    
    % Only enqueue if ETA is within the clearance+safety horizon
    if eta > (t_despeje + in_safety_margin), continue; end
    
    % Enqueue (grow queue if needed)
    q_tail = q_tail + 1;
    if q_tail > numel(queue_ids)
        queue_ids(end+in_queue_growth) = 0;
        queue_dists(end+in_queue_growth) = 0;
    end
    queue_ids(q_tail) = sh_id;
    queue_dists(q_tail) = d;
end

% ---- SEED PHASE 2: if no SH found on current link, try next-link adjacency ----
if q_tail == 0 && ~isempty(sh_in_curr_link)
    remaining_dist = max(0, in_length_2d_of_links(curr_link_idx) - in_curr_ev_pos);
    last_sh_id = sh_in_curr_link(end);
    
    if last_sh_id >= 1 && last_sh_id <= max_sh_id
        sh_idx = sh_lookup(last_sh_id);
        if sh_idx > 0
            % Use the adjacency of the last signal head on the current link
            adj_ids = in_signal_heads_adjacency_list{sh_idx};
            adj_dists = in_signal_heads_distances{sh_idx};
            for j = 1:numel(adj_ids)
                adj_id = adj_ids(j);
                if adj_id < 1 || adj_id > max_sh_id, continue; end
                if sh_lookup(adj_id) == 0, continue; end
                
                dist_adj = adj_dists(j);
                d_total = remaining_dist + dist_adj;
                eta_total = d_total / speed_mps;
                
                idx_link_adj = sh_to_link_idx(adj_id);
                if idx_link_adj > 0
                    t_despeje = in_clearance_time_of_links(idx_link_adj);
                else
                    t_despeje = 0;
                end
                
                % Enqueue if within tolerance or within ETA horizon
                if dist_adj <= in_conn_tolerance || eta_total <= (t_despeje + in_safety_margin)
                    q_tail = q_tail + 1;
                    if q_tail > numel(queue_ids)
                        queue_ids(end+in_queue_growth) = 0;
                        queue_dists(end+in_queue_growth) = 0;
                    end
                    queue_ids(q_tail) = adj_id;
                    queue_dists(q_tail) = d_total;
                end
            end
        end
    end
end

% ---- SEED PHASE 3: if current link has NO signal heads at all ----
if q_tail == 0 && isempty(sh_in_curr_link)
    remaining_dist = max(0, in_length_2d_of_links(curr_link_idx) - in_curr_ev_pos);
    % Scan all signal heads in the network for reachability
    for k = 1:num_sh
        sh_id = in_no_of_signal_heads(k);
        d_total = remaining_dist + in_pos_of_signal_heads(k);
        eta_total = d_total / speed_mps;
        
        idx_link = sh_to_link_idx(sh_id);
        if idx_link > 0
            t_despeje = in_clearance_time_of_links(idx_link);
        else
            t_despeje = 0;
        end
        
        if eta_total <= (t_despeje + in_safety_margin)
            q_tail = q_tail + 1;
            if q_tail > numel(queue_ids)
                queue_ids(end+in_queue_growth) = 0;
                queue_dists(end+in_queue_growth) = 0;
            end
            queue_ids(q_tail) = sh_id;
            queue_dists(q_tail) = d_total;
        end
    end
end

% ==== BFS EXPANSION: propagate through signal head adjacency graph ====
q_head = 1;
seen = false(1, max_sh_id);
sp_list = zeros(1, in_sp_capacity);
sp_count = 0;

while q_head <= q_tail
    % Dequeue current signal head and its accumulated distance
    curr_sh_id  = queue_ids(q_head);
    curr_dist   = queue_dists(q_head);
    q_head = q_head + 1;
    
    % Validate and skip if already visited
    if curr_sh_id < 1 || curr_sh_id > max_sh_id, continue; end
    curr_sh_idx = sh_lookup(curr_sh_id);
    if curr_sh_idx == 0 || seen(curr_sh_id), continue; end
    
    % Mark as visited and add to SP result list
    seen(curr_sh_id) = true;
    sp_count = sp_count + 1;
    if sp_count > numel(sp_list), sp_list(end+in_sp_capacity) = 0; end
    sp_list(sp_count) = curr_sh_id;
    
    % Expand to all downstream adjacent signal heads
    adj_ids = in_signal_heads_adjacency_list{curr_sh_idx};
    adj_dists = in_signal_heads_distances{curr_sh_idx};
    
    for j = 1:numel(adj_ids)
        next_sh_id = adj_ids(j);
        if next_sh_id < 1 || next_sh_id > max_sh_id, continue; end
        if seen(next_sh_id), continue; end
        
        % Compute cumulative distance and ETA to the next signal head
        dist_next = adj_dists(j);
        d_total = curr_dist + dist_next;
        eta_total = d_total / speed_mps;
        
        % Resolve the clearance time of the next signal head's link
        idx_link_next = sh_to_link_idx(next_sh_id);
        if idx_link_next > 0
            t_despeje = in_clearance_time_of_links(idx_link_next);
        else
            t_despeje = 0;
        end
        
        % Enqueue if within tolerance or within ETA horizon
        if dist_next <= in_conn_tolerance || eta_total <= (t_despeje + in_safety_margin)
            q_tail = q_tail + 1;
            if q_tail > numel(queue_ids)
                queue_ids(end+in_queue_growth) = 0;
                queue_dists(end+in_queue_growth) = 0;
            end
            queue_ids(q_tail) = next_sh_id;
            queue_dists(q_tail) = d_total;
        end
    end
end

% Trim and deduplicate the SP result list
out_sp_new = sp_list(1:sp_count);
if ~isempty(out_sp_new)
    out_sp_new = unique(out_sp_new, 'stable');
end

end
