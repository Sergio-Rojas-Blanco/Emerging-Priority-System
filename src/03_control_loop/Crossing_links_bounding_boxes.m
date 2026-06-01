% DESCRIPTION: Detects geometric intersections between non-adjacent links using
%              axis-aligned bounding box (AABB) filtering and exact segment-segment
%              intersection tests. Returns pairs of crossing link IDs and their
%              intersection distances along each link's polyline.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [id_crossing_links, dist_crossing_links] = Crossing_links_bounding_boxes(in_coords_of_links, in_adjacency_list_of_links, in_no_of_links, in_from_link_of_links, in_to_link_of_links, in_is_conn_of_links, in_from_pos_of_links, in_conn_tolerance)

num_links = numel(in_no_of_links);

% Initialize output cell arrays: one cell per link, each will hold
% the IDs and distances of crossing links detected for that link
id_crossing_links = cell(1, num_links);
dist_crossing_links = cell(1, num_links);

% Pre-allocate bounding box matrix [xmin, ymin, xmax, ymax] per link
bb = nan(num_links,4);
valid = false(1, num_links);
coords_xy = cell(1, num_links);     % XY coordinates per link
cum_dists = cell(1, num_links);     % Cumulative distance along each link polyline
seg_bboxes = cell(1, num_links);    % Per-segment bounding boxes for narrow-phase check

% Build per-link geometry: bounding box, cumulative arc-length, segment AABBs
for i = 1:num_links
    coords = in_coords_of_links{i};
    % Skip links with fewer than 2 polyline points (no segments)
    if isempty(coords) || size(coords,1) < 2, continue; end
    xy = coords(:,1:2);
    coords_xy{i} = xy;
    % Compute the global AABB for this link
    mn = min(xy, [], 1); mx = max(xy, [], 1);
    bb(i,:) = [mn(1), mn(2), mx(1), mx(2)];
    valid(i) = true;
    % Compute cumulative distance along the polyline (for locating intersection points)
    d = sqrt(sum(diff(xy).^2, 2));
    cum_dists{i} = [0; cumsum(d)];
    % Compute per-segment bounding boxes [xmin, ymin, xmax, ymax] for narrow-phase
    seg_bboxes{i} = [min(xy(1:end-1,:), xy(2:end,:)), max(xy(1:end-1,:), xy(2:end,:))];
end

% Build a sparse symmetric adjacency matrix from the link adjacency list.
% Adjacent links (connected by connectors) are excluded from crossing detection
% because they share a physical junction, not a geometric crossing.
from_idx = []; to_idx = [];
for i = 1:num_links
    lst = double(in_adjacency_list_of_links{i});
    if isempty(lst), continue; end
    % Map neighbour IDs to their indices in the link array
    [~, loc] = ismember(lst, double(in_no_of_links));
    valid_locs = loc(loc > 0);
    k = numel(valid_locs);
    from_idx(end+1:end+k) = i;
    to_idx(end+1:end+k) = valid_locs(:)'; % Force row vector
end
% Symmetric sparse matrix: if i->j exists, also mark j->i
adj_map = sparse([from_idx, to_idx], [to_idx, from_idx], 1, num_links, num_links);

% Pre-compute connector from/to indices and from-positions for equivalence checks.
% A geometric crossing that coincides with an existing connector is not a true
% crossing conflict — it is a physical junction already captured by adjacency.
conn_mask = (in_is_conn_of_links == 1);
[~, conn_f_idx] = ismember(double(in_from_link_of_links(conn_mask)), double(in_no_of_links));
[~, conn_t_idx] = ismember(double(in_to_link_of_links(conn_mask)), double(in_no_of_links));
conn_f_pos = in_from_pos_of_links(conn_mask);

% Filter out connectors with unresolved from/to indices
valid_conn = (conn_f_idx > 0 & conn_t_idx > 0);
conn_f_idx = conn_f_idx(valid_conn);
conn_t_idx = conn_t_idx(valid_conn);
conn_f_pos = conn_f_pos(valid_conn);

% ---- Main crossing detection: broad-phase AABB + narrow-phase segment test ----
for i = 1:num_links-1
    if ~valid(i), continue; end
    bbi = bb(i,:);

    % Broad-phase: check AABB overlap between link i and all links j > i
    bb_check = ~(bbi(1) > bb(i+1:end,3) | bbi(3) < bb(i+1:end,1) | bbi(2) > bb(i+1:end,4) | bbi(4) < bb(i+1:end,2));
    overlap_mask = valid(i+1:end) & bb_check'; % Both are 1 x K row vectors
    candidates = find(overlap_mask) + i;       % Candidate link indices with AABB overlap

    % Narrow-phase: for each candidate, test all segment pairs
    for j = candidates
        % Skip adjacent links (they share a junction, not a crossing)
        if adj_map(i,j) > 0, continue; end

        found_i = []; found_j = [];
        xi = coords_xy{i}; xj = coords_xy{j};
        sbb_i = seg_bboxes{i}; sbb_j = seg_bboxes{j};

        % Iterate over segments of link i
        for si = 1:size(sbb_i,1)
            sbi = sbb_i(si,:);
            % Filter segments of link j whose AABB overlaps with segment si
            mask = ~(sbi(1) > sbb_j(:,3) | sbi(3) < sbb_j(:,1) | sbi(2) > sbb_j(:,4) | sbi(4) < sbb_j(:,2));
            sj_list = find(mask)';
            
            % Test exact segment-segment intersection for each overlapping pair
            for sj = sj_list
                [intersects, ipt] = segment_intersection(xi(si,:), xi(si+1,:), xj(sj,:), xj(sj+1,:));
                if intersects
                    % Compute intersection distance along link i and link j polylines
                    di = cum_dists{i}(si) + norm(ipt - xi(si,:));
                    dj = cum_dists{j}(sj) + norm(ipt - xj(sj,:));

                    % Connector equivalence check: if this intersection coincides
                    % with an existing connector (within tolerance), discard it
                    is_equiv = false;
                    m1 = (conn_f_idx == i & conn_t_idx == j);
                    if any(m1) && min(abs(di - conn_f_pos(m1))) <= in_conn_tolerance, is_equiv = true; end
                    if ~is_equiv
                        m2 = (conn_f_idx == j & conn_t_idx == i);
                        if any(m2) && min(abs(dj - conn_f_pos(m2))) <= in_conn_tolerance, is_equiv = true; end
                    end

                    % Record only genuine crossings (not connector junctions)
                    if ~is_equiv
                        found_i(end+1) = di; found_j(end+1) = dj; %#ok<AGROW> 
                    end
                end
            end
        end

        % Store crossing results symmetrically for both links
        if ~isempty(found_i)
            id_i = in_no_of_links(i); id_j = in_no_of_links(j);
            id_crossing_links{i} = [id_crossing_links{i}, repmat(id_j, 1, numel(found_i))];
            dist_crossing_links{i} = [dist_crossing_links{i}, found_i];
            id_crossing_links{j} = [id_crossing_links{j}, repmat(id_i, 1, numel(found_j))];
            dist_crossing_links{j} = [dist_crossing_links{j}, found_j];
        end
    end
end
end

% ---- Local function: exact 2D segment-segment intersection ----
% DESCRIPTION: Tests whether two 2D line segments (p1-p2) and (p3-p4) intersect.
%              Uses the parametric form: p1 + t*(p2-p1) and p3 + s*(p4-p3).
%              Returns the intersection point if 0 <= t,s <= 1 (within tolerance).
function [intersects, ipt] = segment_intersection(p1, p2, p3, p4)
    % Direction vectors for each segment
    d1 = p2 - p1; d2 = p4 - p3; dp = p1 - p3;
    % Compute the 2x2 determinant (cross product of direction vectors)
    det = d1(1)*d2(2) - d1(2)*d2(1);
    % If determinant is near zero, segments are parallel or collinear
    if abs(det) < 1e-10
        intersects = false; ipt = []; return;
    end
    % Solve for parametric coordinates t (on segment 1) and s (on segment 2)
    t = (d2(1)*dp(2) - d2(2)*dp(1)) / det;
    s = (d1(1)*dp(2) - d1(2)*dp(1)) / det;
    % Check if both parameters lie within [0, 1] (with small tolerance)
    if t >= -1e-10 && t <= 1+1e-10 && s >= -1e-10 && s <= 1+1e-10
        intersects = true; ipt = p1 + t * d1;
    else
        intersects = false; ipt = [];
    end
end
