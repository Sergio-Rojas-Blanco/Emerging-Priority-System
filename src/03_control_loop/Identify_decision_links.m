% DESCRIPTION: Identifies decision links where the EV confirms its route after
%              a bifurcation. A link is flagged if its predecessor has more than
%              one outgoing neighbour.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_is_decision_link] = Identify_decision_links(in_no_of_links, in_links_adjacency_list)
% IDENTIFY_DECISION_LINKS Identifies links where a route decision is confirmed.
% A link is considered a decision link if its predecessor has more than one exit (bifurcation).
%
% INPUTS:
% - in_no_of_links: Row vector with VISSIM link IDs.
% - in_links_adjacency_list: Row cell where each position [link_idx] contains adjacent link IDs.
%
% OUTPUTS:
% - out_is_decision_link: Logical row vector [link_idx] (true if decision link).

out_is_decision_link = false(1, length(in_no_of_links));
% For each link, check if it has more than one downstream neighbour
for i = 1:length(in_no_of_links)
    adj_ids = in_links_adjacency_list{i};
    % If this link has multiple exits, all its downstream neighbours are decision links
    if length(adj_ids) > 1
        for j = 1:length(adj_ids)
            [~, target_idx] = ismember(adj_ids(j), in_no_of_links);
            out_is_decision_link(target_idx) = true;
        end
    end
end
end
