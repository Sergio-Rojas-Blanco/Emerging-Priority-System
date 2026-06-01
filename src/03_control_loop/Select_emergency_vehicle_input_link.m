% DESCRIPTION: Dynamic control loop module. Executes VISSIM COM interface, real-time telemetry, and Extended Green Wave (SP) / Red Line (SC) priority algorithms.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_link] = Select_emergency_vehicle_input_link(in_vissim, in_no_of_links, in_adjacency_list_of_links, in_use_fixed_link, in_fixed_link)
% SELECT_EMERGENCY_VEHICLE_INPUT_LINK Selects the entry link for the EV
%
% Inputs:
%   in_vissim                  - VISSIM COM object
%   in_no_of_links             - Vector with IDs of all links
%   in_adjacency_list_of_links - Cell array with adjacent links by index
%   in_use_fixed_link          - true to use a fixed link, false for random
%   in_fixed_link              - Fixed link ID to use if in_use_fixed_link is true
%
% Outputs:
%   out_link                   - Selected entry link ID

vehicle_input_links_cell = in_vissim.Net.VehicleInputs.GetMultiAttValues('Link');
if isempty(vehicle_input_links_cell)
    error('No vehicle inputs found in the VISSIM network.');
end
link_ids = cellfun(@str2double, vehicle_input_links_cell(:,2));
[found, loc] = ismember(link_ids, in_no_of_links);
valid_mask = false(size(link_ids));
valid_mask(found) = ~cellfun(@isempty, in_adjacency_list_of_links(loc(found)));
valid_input_indices = find(valid_mask);
if isempty(valid_input_indices)
    error('No valid vehicle inputs with downstream connections found.');
end
fprintf('  [DEBUG] Valid links for EV: %d options -> ', numel(valid_input_indices));
if in_use_fixed_link
    out_link = in_fixed_link;
    fprintf('using fixed=%d\n', in_fixed_link);
else
    rand_k = valid_input_indices(randi(numel(valid_input_indices)));
    out_link = str2double(vehicle_input_links_cell{rand_k, 2});
    fprintf('selected random=%d (out of %s)\n', out_link, mat2str(link_ids(valid_input_indices)'));
end
end
