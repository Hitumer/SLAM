function [h_map,loc_features] = Init_map(map_dim,map_size,num_features,distr_features,file_loc)

%% Init_map : initialize map
%
% This function initialize the map by populating it with features
%
% Syntax:
%    [h_map] = Init_map(map_dim,map_size,num_features,distr_features)
%
% Input arguments:
%    map_dim        - Map dimensionality 
%    map_size       - Size of map [m]
%    num_features   - Number of features required
%    distr_features - Distribution of features
%    file_loc       - Path and name of file containing the map
%
% Output arguments:
%    h_map          - Handle of figure object
%    loc_features   - Matrix of feature locations and unique ID

% ----------------------------------------------------------------------
% File.....: Init_map.m
% Date.....: 02-Mar-2015
% Version..: 1.0 - Last update by G.Giorgi, 21-Apr-2015
% Author...: G. Giorgi
%            Institute for Communications and Navigation
%            Technische Universität München
% ----------------------------------------------------------------------


%% Features initialization

% Locate features in the map
if distr_features==1                    % Create uniform dstribution of features
    loc_features = unifrnd(0,map_size,num_features,map_dim);        
elseif distr_features==2                % If custom coordinates are provided, read the corresponding file
    aux = load(file_loc);    % Open custom file of 2D/3D features locations
    loc_features = aux.loc_features;
end

% Sort location of features (used to optmize search later in the algorithm)
loc_features=sortrows(loc_features,1);

% Attach unique ID (integer scalar) to each feature
if distr_features==1 
    loc_features=[loc_features [1:size(loc_features,1)]'];
end


% Verify correctness of dimensionality across selected map type and provided features coordinates
if ~(map_dim==(size(loc_features,2)-1))
    num_features = size(loc_features,1);
    warning('Dimensionality of map and custom feature locations seem to mismatch. Check initialization file.')
    return
end

% Visualize map 
figure;
if map_dim==2
    plot(loc_features(:,1),loc_features(:,2),'black+');
    set(gca,'FontSize',12)
    set(get(gca,'XLabel'),'String','X')
    set(gca,'XLim',[-0.1*(map_size),map_size+0.1*(map_size)]);
    set(get(gca,'YLabel'),'String','Y')
    set(gca,'YLim',[-0.1*(map_size),map_size+0.1*(map_size)]);
    %axis equal
elseif map_dim==3
    plot3(loc_features(:,1),loc_features(:,2),loc_features(:,3),'black+');
    set(gca,'FontSize',12)
    set(get(gca,'XLabel'),'String','X')
    set(get(gca,'YLabel'),'String','Y')
    set(get(gca,'ZLabel'),'String','Z')
    axis equal
end
h_map=gcf;
axis equal



