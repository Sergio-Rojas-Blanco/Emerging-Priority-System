% DESCRIPTION: Builds the link-level adjacency list from the VISSIM network
%              topology. For each link, identifies downstream neighbours
%              (connectors and their target links) and computes connection
%              point positions along the parent link.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_adjacency_list_of_links, out_connection_point_of_adjacent] = Adjacency_list_of_links(in_no_of_links, in_from_link_of_links, in_to_link_of_links, in_from_pos_of_links, in_to_pos_of_links)

    % Normalize inputs to row vectors
    in_no_of_links = in_no_of_links(:)';
    in_from_link_of_links = in_from_link_of_links(:)';
    in_to_link_of_links = in_to_link_of_links(:)';

    n = numel(in_no_of_links);
    out_adjacency_list_of_links = cell(1, n);
    out_connection_point_of_adjacent = cell(1, n);

    % Identify connectors (from link is defined)
    conn_idx = find(~isnan(in_from_link_of_links));
    if isempty(conn_idx)
        return;
    end

    conn_ids = in_no_of_links(conn_idx);         % Connector IDs (match positions conn_idx)
    from_ids = in_from_link_of_links(conn_idx);
    to_ids = in_to_link_of_links(conn_idx);

    % ID -> index mapping (computed once)
    [in_universe_mask, id2idx] = ismember(in_no_of_links, in_no_of_links); %#ok<ASGLU> % trivially true
    % To look up any ID in in_no_of_links use ismember(toID, in_no_of_links)
    % Precompute lookup for all IDs we need (from/to/conn_ids)
    [~, map_from_ids] = ismember(from_ids, in_no_of_links);
    [~, map_to_ids]   = ismember(to_ids,   in_no_of_links);
    [~, map_conn_ids] = ismember(conn_ids, in_no_of_links); % should match conn_idx

    % Direct adjacency from connectors to their target (0 if loop, empty if NaN)
    % Assign by index (map_conn_ids) in out_adjacency_list_of_links
    for k = 1:numel(conn_idx)
        cIdx = map_conn_ids(k); % index in 1..n
        if isnan(to_ids(k))
            out_adjacency_list_of_links{cIdx} = [];
        elseif from_ids(k) == to_ids(k)
            out_adjacency_list_of_links{cIdx} = 0;
        else
            out_adjacency_list_of_links{cIdx} = to_ids(k);
        end
    end

    % Group connectors by source link (parent index = index of from_id in in_no_of_links)
    valid_parent_mask = map_from_ids > 0;
    if any(valid_parent_mask)
        parent_idx = map_from_ids(valid_parent_mask);   % source indices in 1..n
        child_ids  = conn_ids(valid_parent_mask);       % child connector IDs
        % accumarray to group child_ids by parent index
        grouped = accumarray(parent_idx', child_ids', [n 1], @(v){v}, {});
        nonempty_parents = find(~cellfun('isempty', grouped));
        for p = nonempty_parents'
            out_adjacency_list_of_links{p} = [out_adjacency_list_of_links{p}, grouped{p}'];
        end
    end

    % Normalize positions
    if nargin < 4 || isempty(in_from_pos_of_links)
        in_from_pos_of_links = nan(1,n);
    else
        in_from_pos_of_links = reshape(double(in_from_pos_of_links),1,[]);
    end
    if nargin < 5 || isempty(in_to_pos_of_links)
        in_to_pos_of_links = nan(1,n);
    else
        in_to_pos_of_links = reshape(double(in_to_pos_of_links),1,[]);
    end

    % Prepare global ID -> index mapping for subsequent vectorized lookups
    % Build lookup table using ismember over all present IDs (in_no_of_links is the domain)
    % For speed: could create a sparse vector lookup (if IDs are small integers).
    % We use ismember to convert neighbour IDs to indices when needed.

    % Process each link: vectorize as much as possible
    for i = 1:n
        neighs = out_adjacency_list_of_links{i};
        if isempty(neighs)
            out_connection_point_of_adjacent{i} = zeros(1,0);
            continue;
        end
        neighs = double(neighs(:))'; % row
        % neighbours with value 0 -> NaN directly
        conn_pos = nan(1, numel(neighs));

        % Indices of neighbours in in_no_of_links (0 if not found)
        [ism, neighbor_idx] = ismember(neighs, in_no_of_links);
        % non-existent neighbours -> NaN (already by default)
        valid_neigh = ism & (neighs ~= 0);

        if any(valid_neigh)
            idxs = neighbor_idx(valid_neigh); % indices in 1..n
            % neighbours that are connectors: have in_from_link_of_links(idx) not NaN
            is_connector = ~isnan(in_from_link_of_links(idxs));
            % For connectors that are the same element as source (idx == i),
            % use in_to_pos_of_links(idx); for other connectors use in_from_pos_of_links(idx).
            connector_idxs = idxs(is_connector);
            if ~isempty(connector_idxs)
                % determine which of those have idx == i
                same_as_source = (connector_idxs == i);
                if any(same_as_source)
                    conn_pos(valid_neigh) = conn_pos(valid_neigh); % keep NaN by default
                    % assign matching ones
                    sel = find(valid_neigh);
                    sel_conn = sel(is_connector);
                    sel_same = sel_conn(same_as_source);
                    if ~isempty(sel_same)
                        conn_pos(sel_same) = in_to_pos_of_links(connector_idxs(same_as_source));
                    end
                    sel_other = sel_conn(~same_as_source);
                    if ~isempty(sel_other)
                        conn_pos(sel_other) = in_from_pos_of_links(connector_idxs(~same_as_source));
                    end
                else
                    % none equals i: all take in_from_pos
                    sel = find(valid_neigh);
                    sel_conn = sel(is_connector);
                    if ~isempty(sel_conn)
                        conn_pos(sel_conn) = in_from_pos_of_links(connector_idxs);
                    end
                end
            end
            % valid neighbours that are not connectors -> NaN (already by default)
        end

        % neighbours with value == 0 -> conn_pos = NaN (already by default)
        out_connection_point_of_adjacent{i} = conn_pos;
    end
end
