% DESCRIPTION: Configures the traffic congestion level by scaling VISSIM vehicle
%              input volumes by the corresponding congestion factor.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function [out_original_volume] = Configure_traffic_volume(in_vissim, in_congestion_level)
% CONFIGURE_TRAFFIC_VOLUME Configures congestion level by adjusting volumes.
%
% Inputs:
%   in_vissim           - VISSIM COM object
%   in_congestion_level - Congestion level ('FREE_FLOW', 'LIGHT', 'MODERATE',
%                         'HEAVY', 'SEVERE', 'EXTREME')
%
% Outputs:
%   out_original_volume - Vector with original volumes before adjustment

% Retrieve current vehicle input volumes from VISSIM COM
vehicle_input_volume_cell_1 = in_vissim.Net.VehicleInputs.GetMultiAttValues('Volume(1)');
if isempty(vehicle_input_volume_cell_1)
    error('No vehicle inputs found in the VISSIM network.');
end

% Store original volumes before scaling
out_original_volume = cell2mat(vehicle_input_volume_cell_1(:,2));

% Map congestion level string to its numeric scaling factor
volume_levels = struct('FREE_FLOW', 0.4, 'LIGHT', 0.7, 'MODERATE', 1.0, 'HEAVY', 1.3, 'SEVERE', 1.6, 'EXTREME', 2.0);
volume_factor = volume_levels.(in_congestion_level);

% Scale all vehicle input volumes by the congestion factor
vehicle_input_volume_new = out_original_volume * volume_factor;

% Build the (ID, value) pair array required by VISSIM SetMultiAttValues
nInputs = size(vehicle_input_volume_cell_1, 1);
new_pairs = cell(nInputs, 2);
for k = 1:nInputs
    new_pairs{k,1} = vehicle_input_volume_cell_1{k,1};
    if k <= numel(vehicle_input_volume_new)
        new_pairs{k,2} = vehicle_input_volume_new(k);
    else
        new_pairs{k,2} = vehicle_input_volume_cell_1{k,2};
    end
end

% Apply the scaled volumes back to VISSIM via COM
in_vissim.Net.VehicleInputs.SetMultiAttValues('Volume(1)', new_pairs);
end
