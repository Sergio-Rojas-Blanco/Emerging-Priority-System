% DESCRIPTION: Merges real signal heads with virtual (fictitious) signal heads into
%              an extended signal head network. Supports selective inclusion of
%              endpoint and/or crossing virtual signal heads via boolean flags.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_no, out_pos, out_link, out_sh_of_links] = Unify_ficticial_signal_heads(in_no_real, in_pos_real, in_link_real, in_sh_of_links_real, in_no_end, in_pos_end, in_link_end, in_sh_of_links_end, in_no_cross, in_pos_cross, in_link_cross, in_sh_of_links_cross, in_no_of_links, inc_end, inc_cross)

% Start with the real signal head arrays as the base extended set
out_no = double(in_no_real);
out_pos = double(in_pos_real);
out_link = double(in_link_real);
out_sh_of_links = in_sh_of_links_real;
num_links = numel(in_no_of_links);

% ---- Append endpoint virtual signal heads if requested ----
if inc_end && ~isempty(in_no_end)
    out_no = [out_no, double(in_no_end)];
    out_pos = [out_pos, double(in_pos_end)];
    out_link = [out_link, double(in_link_end)];
    % Register endpoint signal heads in the per-link cell array
    for i = 1:num_links
        val = double(in_sh_of_links_end{i}); 
        if ~isempty(val) && any(val > 0)
            out_sh_of_links{i} = [double(out_sh_of_links{i}), val];
        end
    end
end

% ---- Append crossing virtual signal heads if requested ----
if inc_cross && ~isempty(in_no_cross)
    % Flatten crossing cell arrays: take the first position/link for topological use
    cross_pos_flat = cellfun(@(x) x(1), in_pos_cross);
    cross_link_flat = cellfun(@(x) x(1), in_link_cross);
    
    out_no = [out_no, double(in_no_cross)];
    out_pos = [out_pos, double(cross_pos_flat)];
    out_link = [out_link, double(cross_link_flat)];
    
    % Register crossing signal heads in the per-link cell array
    for i = 1:num_links
        val = double(in_sh_of_links_cross{i});
        if ~isempty(val)
            out_sh_of_links{i} = [double(out_sh_of_links{i}), val];
        end
    end
end

% ---- Remove duplicates and clean empty entries ----
for i = 1:num_links
    if ~isempty(out_sh_of_links{i})
        out_sh_of_links{i} = unique(out_sh_of_links{i}, 'stable');
    else
        out_sh_of_links{i} = zeros(1,0);
    end
end
end
