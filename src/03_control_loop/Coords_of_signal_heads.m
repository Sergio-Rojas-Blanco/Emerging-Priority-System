% DESCRIPTION: Dynamic control loop module. Executes VISSIM COM interface, real-time telemetry, and Extended Green Wave (SP) / Red Line (SC) priority algorithms.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function out_coords_of_signal_heads = Coords_of_signal_heads(in_no_of_links, in_no_of_signal_heads, in_link_of_signal_heads,in_pos_of_signal_heads, in_coords_of_links, in_length_2D_of_links)

    nSignals = numel(in_no_of_signal_heads);
    out_coords_of_signal_heads = nan(nSignals,2);

    % Simple map: obtain 1..nlinks indices for each link ID
    [hasIdx, linkIdx] = ismember(in_link_of_signal_heads, in_no_of_links);
    validSigIdx = find(hasIdx);
    if isempty(validSigIdx)
        return
    end

    nlinks = numel(in_no_of_links);

    % Precompute nodes, segment lengths, and edges only for used links
    usedLinks = unique(linkIdx(validSigIdx));
    xs = cell(1,nlinks);
    ys = cell(1,nlinks);
    segL = cell(1,nlinks);
    edges = cell(1,nlinks);

    for k = usedLinks(:).'
        coords = in_coords_of_links{k};      % double Nx3
        x = coords(:,1);
        y = coords(:,2);
        dx = diff(x); dy = diff(y);
        L = sqrt(dx.^2 + dy.^2);
        xs{k} = x;
        ys{k} = y;
        segL{k} = L;
        edges{k} = [0; cumsum(L)];
    end

    % Group signals by link index and process per group
    % linkIdx(validSigIdx) gives the index for each valid signal
    [grp, ~, ic] = unique(linkIdx(validSigIdx));
    sigList = validSigIdx;
    for gi = 1:numel(grp)
        k = grp(gi);                    % link index
        sigs = sigList(ic == gi);       % indices of signals on this link

        pos = in_pos_of_signal_heads(sigs);
        pos = pos(:);

        total_len = in_length_2D_of_links(k);
        pos = max(0, min(pos, total_len));  % clamp

        ed = edges{k};
        npts = numel(ed);

        is_end = pos == total_len;
        seg = nan(size(pos));
        seg(~is_end) = discretize(pos(~is_end), ed, 'IncludedEdge','right');
        seg(isnan(seg)) = 1;
        seg(is_end) = npts - 1;
        seg = max(1, min(npts-1, seg));
        seg = seg(:);

        dist_in_seg = pos - ed(seg);
        Lseg = segL{k};
        Lseg_per_sig = Lseg(seg);
        t = zeros(size(dist_in_seg));
        nz = Lseg_per_sig > 0;
        t(nz) = dist_in_seg(nz) ./ Lseg_per_sig(nz);

        xnodes = xs{k}; ynodes = ys{k};
        x1 = xnodes(seg); x2 = xnodes(seg+1);
        y1 = ynodes(seg); y2 = ynodes(seg+1);

        x = x1 + t.*(x2 - x1);
        y = y1 + t.*(y2 - y1);

        out_coords_of_signal_heads(sigs,:) = [x(:), y(:)];
    end
end
