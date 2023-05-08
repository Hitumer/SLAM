%% Init : initialize simulation parameters
%
% This script contains the whole set of parameters used by the VisNavSim
% tool
%
% Syntax:
%    Init
%
% Input arguments:
%    none
%
% Output arguments:
%    none

% ----------------------------------------------------------------------
% File.....: VisNavSim.m
% Date.....: 02-Mar-2015
% Version..: 1.0 - Last update by G.Giorgi, 11-May-2015
% Author...: G. Giorgi
%            Institute for Communications and Navigation
%            Technische Universität München
% ----------------------------------------------------------------------



%% Map parameters
    map_dim = 2;            % Dimensionality of map (2=2D ; 3=3D)
    map_size = 10;          % Size of map (in meters). This value indicates the extension of each dimension of the map (e.g., square side in 2D, cube side in 3D)
    map_known = 0;          % Map known (1) or unknown (0, VSLAM)

%% Features map   
    distr_features = 1;     % Distribution of features: 1=random uniform ; 2=custom (read set of 2D/3D locations)
    if distr_features==1
        num_features = 60; % Number of features to be simulated    
        file_loc = '';
    else
        %file_loc = 'Specify here name of Matlab matrix containing the custom 2D/3D locations of the features';
        %file_loc = 'map1.mat';
        file_loc = 'map2.mat';
        num_features = 5; % Number of features in the map
    end   
    
    
    
%% Simulation parameters
    % Time
    dt=0.4;       % Time interval [s]
    T_0=0;      % Initial time stamp [s]
    T_end=60;   % Final time stamp [s]
    
    % Agent trajectory
    sel_type_traj = 1;  % Trajectory type selection:
                        %  1: squares (planar)  -  Set side below
                        %  2: circles (planar)  -  Set radius below
                        %  3: other.. (TBD)
    sel_dyn_traj = 1;   % Trajectory dynamic selection:
                        %  1: constant velocity  -  Set value below
                        %  2: "sawtooth" acceleration - Set parameters below
                        %  3: other.. (TBD)
    
                        
    % Size of trajectory
    if sel_type_traj == 1
        ref_traj = [1,1]';  % Bottom left anchor point of square trajectory [m] (considering axes orientation)
                            % WARNING: must be coherent with map dimensionality
        ext_traj = 7.0;     % Square side (side parallel to axes) [m]
    elseif sel_type_traj == 2
        ref_traj = [5,5]';  % Center of circular trajectory [m]
                            % WARNING: must be coherent with map dimensionality
        ext_traj = 4;       % Radius of circle [m]
    else
        error('Trajectory parameters not correctly initialized. Check Init.m file');
    end
    
    % Dynamics of trajectory
    if sel_dyn_traj == 1
        dyn_traj = 1;                % Constant velocity along the trajectory perimeter [m/s]
    elseif sel_dyn_traj == 2
        dyn_traj(1:2) = [0.1,0.1]';    % Constant positive/negative acceleration value [m/s^2]
        dyn_traj(3:4) = [0,0]';        % Inferior limit of velocity before inversion [m/s]
        dyn_traj(5:6) = [5,5]';        % Superior limit of velocity before inversion [m/s]
    else
        error('Trajectory parameters not correctly initialized. Check Init.m file');
    end
    
    % Initialize linear dynamics 
    % WARNING: must be coherent with map dimensionality:
    % 2D: position (x2), velocity (x2), acceleration (x2)
    % 3D: position (x3), velocity (x3), acceleration (x3)
    if sel_type_traj == 1       % Square trajectory
        init_lin_dyn = [ref_traj',0,dyn_traj,0,0]'; % WARNING: must be coherent with intial heading!
    elseif sel_type_traj == 2   % Circular trajectory
        init_lin_dyn = [ref_traj(1)-ext_traj,ref_traj(2),0,dyn_traj,0,0]'; % WARNING: must be coherent with intial heading!
    else
        error('Trajectory parameters not correctly initialized. Check Init.m file');
    end
    
    % Initialize rotational dynamics
    % WARNING: must be coherent with map dimensionality:
    % 2D: heading (x1), angular velocity (x1), angular acceleration (x1)
    % 3D: heaading, elevation and bank (x3), angular velocity (x3), angular acceleration (x3)
    init_rot_dyn = [pi/2,0,0]';

    
%% Camera parameters
    cam_lock = 1;                   % Camera locking:
                                    %  1: camera locked to point in the direction of the main axis of the agent body 
                                    %  0: free-moving camera (WARNING: need to define a movement pattern, TBD)
    cam_type = 2;                   % Camera type:
                                    %  1: Monocular (TBD)
                                    %  2: Stero rig
    cam_fov  = pi/4;                % Camera field of view [rad] (total aperture angle, smmetric w.r.t. forward direction)
    cam_dov  = 2;                   % Camera depth of view [m] (maximum distance from camera at which a feature can be observed)              
    init_cam = [ref_traj',pi/2];    % Initialize initial camera position and orientation
    sigmas   = 0.4*[0.1,0.1,0.2];       % Nominal noise values:
                                    % 2D: sigma_x,sigma_y,rho
                                    % 3D: sigma_x,sigma_y,sigma_z,rho_xy,rho_xz,rho_yz
    
%% Visualization parameters
    vis_pers_traj=1;        % Toggle current position / full trajectory:
                            % 0: only current position
                            % 1: persistent trajectory 
    % Set corners of a right-facing triangle                            
    triangle_pt = [0.05 -0.05 -0.05;...                             
                   0     0.05 -0.05];
    % Corners of a left-expanding radial cone
    N_pp=10;
    ppU(1:2,1:1:N_pp)=NaN;
    ppD(1:2,1:1:N_pp)=NaN;
    for i_pt=1:1:10
        ppU(1:2,i_pt)=[cam_dov*cos(0.5*cam_fov*(N_pp+1-i_pt)/N_pp) ;  cam_dov*sin(0.5*cam_fov*(N_pp+1-i_pt)/N_pp)];
        ppD(1:2,i_pt)=[cam_dov*cos(0.5*cam_fov*(i_pt)/N_pp)        ; -cam_dov*sin(0.5*cam_fov*(i_pt)/N_pp)];
    end
    cone_pt = [[0;0] ppU ppD];
               
               
%% Kalman filter parameters
    if sel_type_traj == 1      % Square trajectory
        sigma_state_0 = 0;0.001*[1,1,1];      % Initial state uncertainty
        sigma_process = 0.01*[1,1,1];       % Set sigmas for process noise
        sigma_obs     = 0.05*[1,1];         % Set sigmas for observation noise
    
    elseif sel_type_traj == 2   % Circular trajectory
        sigma_state_0 = 0.001*[1];      % Initial state uncertainty
        sigma_process = 0.01*[1];       % Set sigmas for process noise
        sigma_obs     = 0.1*[1,1];     % Set sigmas for observation noise
        
    else
        error('Process/observation noise matrices not correctly initialized. Check Init.m file');
    end
    
    
    
%% Initialize simulation parameters
    t = T_0;                                            % Initialize at given initial time stamp
    count = 0;                                          % Initialize iterations counter
    Agent_trajectory = [init_lin_dyn;init_rot_dyn];     % Initialize agent trajectory
    Cam_trajectory = init_cam;                          % Initialize camera trajectory
    
     if sel_type_traj == 1      % Square trajectory
        Agent_trajectory_KFstate = Agent_trajectory([1,2,7]);                          % Initialize state vector (exact)
        %Agent_trajectory_KFstate =  mvnrnd(Agent_trajectory([1,2,7]),0.1*eye(3))';    % Initialize state vector (random around known)
    elseif sel_type_traj == 2   % Circular trajectory
        Agent_trajectory_KFstate = Agent_trajectory(7);                         % Initialize state vector (exact)
        %Agent_trajectory_KFstate =  mvnrnd(Agent_trajectory(7),0.1)';          % Initialize state vector (random around known)
    else
        error('State vector not correctly initialized. Check Init.m file');
    end
    
    
    Sigma_state   = diag(sigma_state_0.^2);             % Initial state uncertainty
    Sigma_process = diag(sigma_process.^2);             % Process noise
    Sigma_obs = diag(sigma_obs.^2);                     % Observation noise
    
    h_traj(1:ceil((T_end-T_0)/dt))=NaN;                 % Initialize handle trajectory (for plotting)
    h_mapSLAM=NaN;                                      % Initialize handle map being built (for plotting)
    h_camfov(1:ceil((T_end-T_0)/dt))=NaN;               % Initialize handle camera FOV (for plotting)
    h_traj_KFstate(1:ceil((T_end-T_0)/dt))=NaN;         % Initialize handle estimated trajectory (for plotting)
    h_traj_ell(1:ceil((T_end-T_0)/dt))=NaN;             % Initialize handle to trajectory confidence ellipse (for plotting)
    h_map_ell(1:ceil((T_end-T_0)/dt))=NaN;              % Initialize handle to map features confidence ellipse (for plotting)
    
    Est_error_map=[];                                   % Initialize map estimation error
    Est_error_traj=[];                                  % Initialize trajectory estimation error
    
    ID_map_loc=[];                                      % Initialize ID of stored features (for SLAM)
    loc_features_map=[];                                % Initialize map(for SLAM)
    
    % Initialize figure handles:
    %figure(1);  % Main figure, trajectory and features
    %figure(2);  % Estimation errors

               