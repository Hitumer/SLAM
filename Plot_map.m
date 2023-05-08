%% Plot_map : plot map being built
%
% This script enables plotting the map being built
%
% Syntax:
%    Plot_map
%
% Input arguments:
%    none
%
% Output arguments:
%    none

% ----------------------------------------------------------------------
% File.....: Plot_traj.m
% Date.....: 06-May-2015
% Version..: 1.0 - Last update by G.Giorgi, 06-May-2015
% Author...: G. Giorgi
%            Institute for Communications and Navigation
%            Technische Universität München
% ----------------------------------------------------------------------



%% Plot
    % Cancel previous map plot
    if ~isnan(h_mapSLAM)
        for i=1:1:length(h_mapSLAM)
            delete(h_mapSLAM(i));
            delete(h_map_ell(i));
        end
    end
    
    % Plot current map and associated ellipses
    for i=1:1:N_map
        h_mapSLAM(i) = plot(loc_features_map(i,1),loc_features_map(i,2),'red+');    
        % Plot 95% confidence ellipse for estimated trajectory (x-y)
        if sel_type_traj == 1      % Square trajectory
            iQaux = inv(Sigma_state(2+i*2:3+i*2,2+i*2:3+i*2));
        elseif sel_type_traj == 2   % Circular trajectory
            iQaux = inv(Sigma_state(i*2:1+i*2,i*2:1+i*2));
        end
        
        cc=ncx2inv(0.5,map_dim,0); % Value of variable at 95% probability
        [Xell_map,Yell_map] = DrawEllipse(iQaux(1,1),iQaux(1,2),iQaux(2,2),0,0,-cc,loc_features_map(i,1:2));
        h_map_ell(i) = plot(Xell_map,Yell_map);

    end