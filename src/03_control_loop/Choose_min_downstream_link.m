% DESCRIPTION: Selects the downstream link with the minimum ID for deterministic
%              EV routing. Resolves connectors to their destination links and
%              filters by forward reachability relative to the current position.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function out_next_link_id = Choose_min_downstream_link(in_current_link_id, in_current_pos, in_adjacency_list_of_links, in_connection_point_of_adjacent, in_is_conn_of_links, in_link_id_to_index, in_conn_tolerance)
% CHOOSE_MIN_DOWNSTREAM_LINK Selects the downstream link with the minimum ID.
%
% Inputs:
%   in_current_link_id           - Current link ID
%   in_current_pos               - Current position on the link (m)
%   in_adjacency_list_of_links   - Cell array with adjacent links per index
%   in_connection_point_of_adjacent - Cell array with connection points
%   in_is_conn_of_links          - Logical vector indicating if each link is a connector
%   in_link_id_to_index          - Map from link ID to index
%   in_conn_tolerance            - Connection tolerance (m)
%
% Outputs:
%   out_next_link_id             - ID of the next link (NaN if none)

out_next_link_id = NaN;

% Validate current link exists in the index map
if ~in_link_id_to_index.isKey(in_current_link_id)
    return;
end
current_idx = in_link_id_to_index(in_current_link_id);

% Get downstream neighbours of the current link
neighs = in_adjacency_list_of_links{current_idx};
if isempty(neighs)
    return;
end
neighs = reshape(double(neighs), 1, []);
conn_pos_list = in_connection_point_of_adjacent{current_idx};
conn_pos_list = reshape(double(conn_pos_list), 1, []);

candidates = [];
% Evaluate each downstream neighbour
for i = 1:numel(neighs)
    n = neighs(i);
    % Skip null neighbours (loop markers)
    if n == 0
        continue;
    end
    % Filter by connection point: skip connectors departing behind the EV
    if i <= numel(conn_pos_list)
        conn_pos = conn_pos_list(i);
        if ~isnan(conn_pos) && (conn_pos < in_current_pos - in_conn_tolerance)
            continue;
        end
    end
    % Validate neighbour exists in the link map
    if ~in_link_id_to_index.isKey(n)
        continue;
    end
    n_idx = in_link_id_to_index(n);
    % If the neighbour is a connector, resolve to its destination link(s)
    if in_is_conn_of_links(n_idx)
        dests = in_adjacency_list_of_links{n_idx};
        if isempty(dests)
            continue;
        end
        dests = reshape(double(dests), 1, []);
        dests = dests(dests ~= 0);
        if ~isempty(dests)
            candidates = [candidates, dests]; %#ok<AGROW>
        end
    else
        % Regular link: add directly as candidate
        candidates = [candidates, n]; %#ok<AGROW>
    end
end

if isempty(candidates)
    return;
end
% Deterministic selection: pick the candidate with the minimum link ID
out_next_link_id = min(candidates);
end
