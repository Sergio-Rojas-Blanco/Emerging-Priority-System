% DESCRIPTION: Extracts network topology and signal head data from the
%              VISSIM COM interface. Produces LinksData and SignalHeadsData
%              structs consumed by downstream adjacency functions.
% AUTHOR: Sergio Rojas-Blanco | sergio.rojas@uca.es | University of Cádiz
% LICENSE: GNU GPLv3

% This script is used to extract the network information from the Vissim simulation model.
linksObj = Vissim.Net.Links;
all_Links = linksObj.GetAll;
% Single COM call for all attributes including LaneWidth to calculate the total width of the road
linksAttrs = linksObj.GetMultipleAttributes({'No';'LinkBehavType';'DisplayType';'NumLanes';'Length2D';'IsConn';'Fromlink';'FromPos';'ToLink';'ToPos'});

NoOfLinks = cell2mat(linksAttrs(:,1))'; % Get the link numbers as a row vector for easier indexing in the rest of the code
LinkBehavTypeOfLinks = str2double(linksAttrs(:,2))'; % Get the behavior type of the links
DisplayTypeOfLinks = str2double(linksAttrs(:,3))'; % Get the display type of the links
NumLanesOfLinks = cell2mat(linksAttrs(:,4))'; % Get the number of lanes of the links
Length2DOfLinks = cell2mat(linksAttrs(:,5))'; % Get the 2D length of the links
IsConnOfLinks = cell2mat(linksAttrs(:,6))'; % Get the connection status of the links
FromLinkOfLinks = str2double(linksAttrs(:,7))'; % Get the from link of the links
FromPosOfLinks = cell2mat(linksAttrs(:,8))'; % Get the from position of the links
ToLinkOfLinks = str2double(linksAttrs(:,9))'; % Get the to link of the links
ToPosOfLinks = cell2mat(linksAttrs(:,10))'; % Get the to position of the links

num_Links = numel(all_Links);
CoordsOfLinks = cell(1, num_Links);
WidthOfLinks = zeros(1, num_Links);
for i = 1:num_Links
    pts = all_Links{i}.LinkPolyPts;
    coordsAttrs = pts.GetMultipleAttributes({'X';'Y';'Rad'});
    CoordsOfLinks{i} = [cell2mat(coordsAttrs(:,1)), cell2mat(coordsAttrs(:,2)), cell2mat(coordsAttrs(:,3))]; % Get the coordinates of the links
    lanesObj = all_Links{i}.Lanes.GetAll;
    laneWidths = cellfun(@(lan) lan.AttValue('Width'), lanesObj);
    WidthOfLinks(i) = sum(laneWidths); % Get the total width of each link
end

signalheasdObj = Vissim.Net.SignalHeads;
signalheadsAttrs = signalheasdObj.GetMultipleAttributes({'No';'Lane';'Pos';'SG'});  % Single COM call for all attributes to reduce overhead
NoOfSignalHeads = cell2mat(signalheadsAttrs(:,1))'; % Get the number of signal heads
LinkLaneOfSignalHeads = cellfun(@(x) strsplit(x, '-'), signalheadsAttrs(:,2), 'UniformOutput', false);
LinkLaneOfSignalHeads = str2double(vertcat(LinkLaneOfSignalHeads{:})); % Get the link and lane of the signal heads as numeric arrays for easier processing
LinkOfSignalHeads = LinkLaneOfSignalHeads(:,1)'; % Get the link of the signal heads
LaneOfSignalHeads = LinkLaneOfSignalHeads(:,2)'; % Get the lane of the signal heads
SignalHeadsOfLinks = arrayfun(@(lid) NoOfSignalHeads(LinkOfSignalHeads == lid), NoOfLinks, 'UniformOutput', false); % Get the signal heads of each link
PosOfSignalHeads = cell2mat(signalheadsAttrs(:,3))';
SG_SC = signalheadsAttrs(:,4);
col = strrep(cellfun(@char, SG_SC, 'UniformOutput', false), '''', '');
C = regexp(col, '-', 'split');
SCOfSignalHeads = reshape(cellfun(@(x) str2double(x{1}), C), 1, []); % Get the signal controller of the signal heads
SGOfSignalHeads = reshape(cellfun(@(x) str2double(x{2}), C), 1, []); % Get the signal group of the signal heads

% --- LinksData packing ---
LinksData.NoOfLinks = NoOfLinks;
LinksData.LinkBehavTypeOfLinks = LinkBehavTypeOfLinks;
LinksData.DisplayTypeOfLinks = DisplayTypeOfLinks;
LinksData.NumLanesOfLinks = NumLanesOfLinks;
LinksData.Length2DOfLinks = Length2DOfLinks;
LinksData.IsConnOfLinks = IsConnOfLinks;
LinksData.FromLinkOfLinks = FromLinkOfLinks;
LinksData.FromPosOfLinks = FromPosOfLinks;
LinksData.ToLinkOfLinks = ToLinkOfLinks;
LinksData.ToPosOfLinks = ToPosOfLinks;
LinksData.CoordsOfLinks = CoordsOfLinks;
LinksData.WidthOfLinks = WidthOfLinks;

% --- SignalHeadsData packing ---
SignalHeadsData.NoOfSignalHeads = NoOfSignalHeads;
SignalHeadsData.LinkOfSignalHeads = LinkOfSignalHeads;
SignalHeadsData.LaneOfSignalHeads = LaneOfSignalHeads;
SignalHeadsData.PosOfSignalHeads = PosOfSignalHeads;
SignalHeadsData.SCOfSignalHeads = SCOfSignalHeads;
SignalHeadsData.SGOfSignalHeads = SGOfSignalHeads;
SignalHeadsData.SignalHeadsOfLinks = SignalHeadsOfLinks;

clear linksObj all_Links num_Links pts coordsAttrs linksAttrs linksObj signalheasdObj signalheadsAttrs SG_SC col C i lanesObj laneWidths;
