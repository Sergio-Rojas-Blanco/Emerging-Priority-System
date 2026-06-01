% DESCRIPTION: Detects convergence (endpoint) conflicts between real signal heads by
%              identifying pairs that share a common downstream virtual endpoint signal
%              head. Excludes parallel signal heads on the same link within tolerance.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_conflicts] = Conflict_list_of_signal_heads_endpoints(in_adj_ext, in_no_ext, in_no_real, in_link_ext, in_pos_ext, in_tol)

num_real = numel(in_no_real);

% Initialize output: one empty conflict list per real signal head
out_conflicts = repmat({zeros(1,0)}, 1, num_real);

% ---- Build inverted index: target node -> list of source real signal heads ----
% For each real signal head, record all its downstream targets. When multiple
% real signal heads share the same downstream target, they are potential
% convergence conflicts.
target_to_sources = containers.Map('KeyType', 'double', 'ValueType', 'any');
[~, real_idx_map] = ismember(in_no_real, in_no_ext);

for r = 1:num_real
    e_idx = real_idx_map(r);
    if e_idx == 0, continue; end
    targets = in_adj_ext{e_idx};
    % Register this real signal head as a source for each of its downstream targets
    for t = 1:numel(targets)
        tid = targets(t);
        if isKey(target_to_sources, tid)
            target_to_sources(tid) = [target_to_sources(tid), in_no_real(r)];
        else
            target_to_sources(tid) = in_no_real(r);
        end
    end
end

% ---- Resolve convergence conflicts ----
targets = keys(target_to_sources);
for t = 1:numel(targets)
    sources = target_to_sources(targets{t});
    % A convergence conflict requires at least 2 sources sharing the same target
    if numel(sources) < 2, continue; end
    
    % Resolve parent link and position for each source signal head
    [~, s_idxs] = ismember(sources, in_no_ext);
    links = in_link_ext(s_idxs);
    pos = in_pos_ext(s_idxs);
    
    % Pairwise conflict check among all sources of this target
    for i = 1:numel(sources)
        [~, r_idx] = ismember(sources(i), in_no_real);
        for j = i+1:numel(sources)
            % Exclude parallel signal heads: same link and positions within tolerance
            is_parallel = (links(i) == links(j)) && (abs(pos(i) - pos(j)) <= in_tol);
            if ~is_parallel
                % Register convergence conflict symmetrically
                out_conflicts{r_idx}(end+1) = sources(j);
                [~, r_jdx] = ismember(sources(j), in_no_real);
                out_conflicts{r_jdx}(end+1) = sources(i);
            end
        end
    end
end

% ---- Deduplicate conflicts per signal head ----
for i = 1:num_real
    if ~isempty(out_conflicts{i})
        out_conflicts{i} = unique(out_conflicts{i}, 'stable');
    end
end
end
