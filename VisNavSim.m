









%% VisNavSim.m 
%
% This is the main function of the VisNav simulation software. Use this function to access all the functionalities of the simulation.
%
% Syntax:
%    VisNavSim
%
% Input arguments:
%    none
%
% Output arguments:
%    none
%
%
% ----------------------------------------------------------------------
% File.....: VisNavSim.m
% Date.....: 02-Mar-2015
% Version..: 1.0 - Last update by G.Giorgi, 28-Jan-2021
% Author...: G. Giorgi
%            Institute for Communications and Navigation
%            DLR / Technische Universität München 
% ----------------------------------------------------------------------


%% Initialization
Init


%% Create and initialize map and features
[h_map,loc_features] = Init_map(map_dim,map_size,num_features,distr_features,file_loc);


%% Start simulation

while 1
    count = count+1;                                % Update counter
    % Update agent trajectory
    [Agent_trajectory,uk] = Compute_trajectory(Agent_trajectory,map_dim,sel_type_traj,sel_dyn_traj,...
                        ref_traj,ext_traj,dyn_traj,dt);
    
    % Update camera trajectory
    Cam_trajectory = Compute_cam_trajectory(Agent_trajectory,Cam_trajectory,map_dim,cam_lock);
    
    % Extract location of features in view (as index) 
    idx_ex_loc = Extract_idx_features(Cam_trajectory,loc_features(:,1:map_dim),cam_fov,cam_dov,map_dim,num_features);
    
        
    % Simulate feature extraction from camera(s)
    if isnan(idx_ex_loc)
        % If no observed features, returns a NaN value
        loc_features_im = NaN;
    else
        [loc_features_im] = Image_features(Cam_trajectory,loc_features(idx_ex_loc,1:map_dim),map_dim,cam_type,sigmas);
        % Attach ID
        loc_features_im = [loc_features_im loc_features(idx_ex_loc,3)]; %#ok<AGROW>
    end
    
    if sel_type_traj == 1       % Square trajectory
        if map_known==1
            SLAM_EKF_tmp_1 % Simple model (x,y,heading), heading not observable, map known
        else
            SLAM_EKF_tmp_2 % Simple model (x,y,heading), heading not observable, map unknown
        end
        N_map = 0.5*(size(Agent_trajectory_KFstate,1)-3);
            
    elseif sel_type_traj == 2   % Circular trajectory
        if map_known==1
            SLAM_EKF_tmp_3 % Simple model (heading), map known
        else
            SLAM_EKF_tmp_4 % Simple model (heading), map unknown
        end
        N_map = 0.5*(size(Agent_trajectory_KFstate,1)-1);
        
    else
        error('Call to KF routines failed. Check VisNavSim.m file');
    end
    
   
    hold on
    Plot_traj                     % Plot actual trajectory and currently observed features
    if 1
        Plot_map                  % Plot current estimated map (VLSAM)
    end
   
 
    
    t=t+dt;                       % Update time
    
    if t > T_end; break; end     % Exit at set time
    
    
end
