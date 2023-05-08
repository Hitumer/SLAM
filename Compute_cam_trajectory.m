function [Cam_trajectory] = Compute_cam_trajectory(Agent_trajectory,Cam_trajectory,map_dim,cam_lock)

%% Compute_cam_trajectory : compute camera trajectory
%
% This function computes the camera isntantaneous position based on the
% agent position and the camera paraemters specified in input
%
% Syntax:
%    [Cam_trajectory] = Compute_cam_trajectory(Agent_trajectory,Cam_trajectory,map_dim,cam_lock)
%
% Input arguments:
%    Agent_trajectory   - Vector of agent dynamics parameters
%    Cam_trajectory     - Vector of camera dynamic parameters
%    map_dim            - Dimensionality of map (see Init.m)
%    cam_lock           - Camera lock to agent trajectory (see Init.m)
%
% Output arguments:
%    Cam_trajectory     - Vector of camera dynamics parameters after time
%                          interval dt based on selected trajectory/camera parameters

% ----------------------------------------------------------------------
% File.....: Compute_cam_trajectory.m
% Date.....: 09-Mar-2015
% Version..: 1.0 - Last update by G.Giorgi, 10-Mar-2015
% Author...: G. Giorgi
%            Institute for Communications and Navigation
%            Technische Universität München
% ----------------------------------------------------------------------


%% Algorithm


    
if map_dim==2               % IF 2D
    
    if cam_lock==1          % IF camera locked to agent trajectory
        Cam_trajectory = [Agent_trajectory(1),Agent_trajectory(2),Agent_trajectory(7)]; % Copy locaton and heading
    else   
        disp('Still to do')
        return
    end
    
end




