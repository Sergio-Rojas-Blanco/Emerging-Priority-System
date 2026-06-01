% DESCRIPTION: Attempts to set the desired next link for the EV via VISSIM COM.
%              Tries multiple attribute name variants for COM version compatibility.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function out_ok = Try_set_vehicle_desired_link(in_vehicle, in_next_link_id)
% TRY_SET_VEHICLE_DESIRED_LINK Attempts to set the next desired link on the EV.
%
% Inputs:
%   in_vehicle      - VISSIM COM vehicle object
%   in_next_link_id - ID of the desired link
%
% Outputs:
%   out_ok          - true if the link was set successfully, false otherwise

out_ok = false;
% Try multiple COM attribute name variants for cross-version compatibility
attrs = {'DesLink','DesiredLink','DestLink','NextLink'};
for k = 1:numel(attrs)
    % Attempt via set()
    try
        set(in_vehicle, 'AttValue', attrs{k}, in_next_link_id);
        out_ok = true;
        return;
    catch
    end
    % Fallback: attempt via SetAttValue()
    try
        in_vehicle.SetAttValue(attrs{k}, in_next_link_id);
        out_ok = true;
        return;
    catch
    end
end
end
