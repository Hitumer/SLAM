function [Agent_trajectory,u] = Compute_trajectory(Agent_trajectory,map_dim,sel_type_traj,sel_dyn_traj,ref_traj,ext_traj,dyn_traj,dt)

%% Compute_trajectory : compute agent trajectory
%
% This function computes the next position of the agent based on the
% previous position and the trajectory type and parameters specified in
% input
%
% Syntax:
%    [Agent_trajectory,u] = Compute_trajectory(Agent_trajectory,sel_type_traj,ext_traj,dyn_traj,dt)
%
% Input arguments:
%    Agent_trajectory   - Vector of agent dynamics parameters
%    map_dim            - Dimensionality of map (see Init.m)
%    sel_type_traj      - Selection of type of trajectory (see Init.m)
%    sel_dyn_traj       - Selection of trajectory dynamic (see Init.m)
%    ref_traj           - Anchor point of selected trajectory (see Init.m)
%    ext_traj           - Spatial extension of selected trajectory [m] (see Init.m)
%    dyn_traj           - Vector of dynamic parameters of selected trajectory (see Init.m)
%    dt                 - Temporal interval [s] (see Init.m)
%
% Output arguments:
%    Agent_trajectory   - Vector of agent dynamics parameters after time
%                          interval dt based on selected trajectory parameters
%    u                  - Trigger command (1: rotate ; 0: no commands)

% ----------------------------------------------------------------------
% File.....: Compute_trajectory.m
% Date.....: 02-Mar-2015
% Version..: 1.0 - Last update by G.Giorgi, 04-Mar-2015
% Author...: G. Giorgi
%            Institute for Communications and Navigation
%            Technische Universität München
% ----------------------------------------------------------------------


%% Algorithm

if sel_type_traj==1         % Square trajectory
    
    if map_dim==2               % IF 2D
        
        if sel_dyn_traj==1          % If constant velocity model
            u = 0;                  % By default, no command            
            if Agent_trajectory(1)==ref_traj(1)                 % First side, going upward
                aux_excess = Agent_trajectory(2)+dyn_traj*dt-(ref_traj(2)+ext_traj);    % Check if next step is outside square
                if aux_excess <= 0   % Simply propagate
                    Agent_trajectory(2)  = Agent_trajectory(2) + dyn_traj*dt;   % Increment y-position
                else                % Correct by turning right
                    Agent_trajectory(2)  = ref_traj(2)+ext_traj;                % Increment y-position up to the corner                    
                    Agent_trajectory(1)  = Agent_trajectory(1) + aux_excess;    % .. and increment x-position
                    Agent_trajectory(7)  = 0;                                   % .. and Heading: 0 deg
                    Agent_trajectory([3,4])  = [dyn_traj,0];                    % .. and velocity (now positive to the left)
                    u = 1;                                                      % .. and trigger turning command
                end
                    
            elseif Agent_trajectory(2)==(ref_traj(2)+ext_traj)  % Second side, going rightward
                aux_excess = Agent_trajectory(1)+dyn_traj*dt-(ref_traj(1)+ext_traj);    % Check if next step is outside square
                if aux_excess <= 0   % Simply propagate
                    Agent_trajectory(1)  = Agent_trajectory(1) + dyn_traj*dt;   % Increment x-position                    
                else                % Correct by turning down
                    Agent_trajectory(1)  = ref_traj(1)+ext_traj;                % Increment x-position up to the corner                    
                    Agent_trajectory(2)  = Agent_trajectory(2) - aux_excess;    % .. and decrement y-position
                    Agent_trajectory(7)  = 3*pi/2;                              % .. and Heading: -90 deg (= +270 deg)
                    Agent_trajectory([3,4])  = [0,-dyn_traj];                   % .. and velocity (now negative down)
                    u = 1;                                                      % .. and trigger turning command
                end
                
            elseif Agent_trajectory(1)==(ref_traj(1)+ext_traj)   % Third side, going downward
                aux_excess = ref_traj(2) - (Agent_trajectory(2)-dyn_traj*dt);   % Check if next step is outside square
                if aux_excess <= 0   % Simply propagate
                    Agent_trajectory(2)  = Agent_trajectory(2) - dyn_traj*dt;   % Decrement y-position                    
                else                % Correct by turning left
                    Agent_trajectory(2)  = ref_traj(2);                         % Decrement x-position up to the corner                    
                    Agent_trajectory(1)  = Agent_trajectory(1) - aux_excess;    % .. and decrement y-position                    
                    Agent_trajectory(7)  = pi;                                  % .. and Heading: 180 deg
                    Agent_trajectory([3,4])  = [-dyn_traj,0];                   % .. and velocity (now negative left)
                    u = 1;                                                      % .. and trigger turning command
                end
                
            elseif Agent_trajectory(2)==ref_traj(2)     % Fourth side, going leftward
                aux_excess = ref_traj(1) - (Agent_trajectory(1)-dyn_traj*dt);   % Check if next step is outside square
                if aux_excess < 0   % Simply propagate
                    Agent_trajectory(1)  = Agent_trajectory(1) - dyn_traj*dt;   % Decrement x-position                    
                else                % Correct by turning up
                    Agent_trajectory(1)  = ref_traj(1);                         % Decrement x-position up to the corner                    
                    Agent_trajectory(2)  = Agent_trajectory(2) + aux_excess;    % .. and increment y-position                   
                    Agent_trajectory(7)  = pi/2;                                % .. and Heading: +90 deg
                    Agent_trajectory([3,4])  = [0,dyn_traj];                    % .. and velocity (now positive up)
                    u = 1;                                                      % .. and trigger turning command
                end
                
            else
                disp('Some problem with trajectory propagation, pheraps the equality conditions for edge selection')
            end
            

        end
    end
elseif sel_type_traj==2  % Square trajectory
    
    if map_dim==2               % IF 2D
        
        if sel_dyn_traj==1          % If constant velocity model
            u = 0;                      % By default, no command
            Agent_trajectory(7)     = Agent_trajectory(7)-dt*dyn_traj/ext_traj;         % Circular trajector (counterclockwise): 
                                                                                        % 1 DOF (angular vel x time interval = angle)
            % Adjust angle so it belongs to [0,2*pi)
            if Agent_trajectory(7) < 0
                Agent_trajectory(7) = 2*pi+Agent_trajectory(7);
            end
    
            Agent_trajectory([1,2]) = [-ext_traj*sin(Agent_trajectory(7)) ;...
                                       ext_traj*cos(Agent_trajectory(7))] + ref_traj;   % Position in x,y                   
        end    
       
    end
    
    
else
    error('Computation of trajectory failed. Check Compute_trajectory.m file');
end
        





