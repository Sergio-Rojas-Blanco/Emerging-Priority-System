% DESCRIPTION: Inserts an emergency vehicle into the VISSIM simulation via COM.
%              Selects the vehicle type by category preference (LGV > CAR) and
%              configures name, speed, color, and interaction attributes.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

function out_emergency_vehicle = Add_emergency_vehicle(Vissim, name, desired_speed, link, color, interaction)
    % Inserts an emergency vehicle in VISSIM with preferred vehicle type selection.
    % Parameters: name (string), desired_speed (km/h), link (numeric ID), color (hex), interaction (bool)

    % Retrieve all vehicle types and their categories from VISSIM COM
    veh_type_info = Vissim.Net.VehicleTypes.GetMultipleAttributes({'No';'Category'});
    if isempty(veh_type_info)
        error('No vehicle types found in the VISSIM model.');
    end

    ids        = veh_type_info(:,1);
    categories = veh_type_info(:,2);

    % Select vehicle type by category preference: LGV first, then CAR
    pref = {'LGV','CAR'};
    idx = [];
    for p = 1:numel(pref)
        idx = find(strcmpi(categories, pref{p}), 1);
        if ~isempty(idx)
            break;
        end
    end
    % Fallback to the first type if no preferred category is found
    if isempty(idx)
        idx = 1;
    end
    emergency_vehicle_type = ids{idx};

    % Validate mandatory parameters
    if isempty(name) || isempty(link)
        error('Name and link are mandatory.');
    end
    if isempty(desired_speed)
        desired_speed = 50;
    end
    if isempty(interaction)
        interaction = false;
    end

    % Insert the vehicle at the start of the specified link (lane 1, position 0)
    out_emergency_vehicle = Vissim.Net.Vehicles.AddVehicleAtLinkPosition(emergency_vehicle_type, link, 1, 0, desired_speed, interaction);

    % Set the vehicle name via COM with explicit error handling
    try
        set(out_emergency_vehicle, 'AttValue', 'Name', name);
    catch ME
        try
            out_emergency_vehicle.SetAttValue('Name', name);
        catch ME2
            warning('AddEmergencyVehicle:SetNameFailed', ...
                'Could not set Name on vehicle object: %s. Error1: %s. Error2: %s', ...
                name, ME.message, ME2.message);
        end
    end

    % Set the vehicle primary color via COM with similar error handling
    try
        set(out_emergency_vehicle, 'AttValue', 'Color1', color);
    catch ME
        try
            out_emergency_vehicle.SetAttValue('Color1', color);
        catch ME2
            warning('AddEmergencyVehicle:SetColorFailed', ...
                'Could not set Color1 on vehicle object: %s. Error1: %s. Error2: %s', ...
                color, ME.message, ME2.message);
        end
    end

end
