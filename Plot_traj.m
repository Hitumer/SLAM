%% Plot_traj : plot agent trajectory and camera FOV
%
% This script enables plotting the agent trajectory and he instantaneous
% camera field of view
%
% Syntax:
%    Plot_traj
%
% Input arguments:
%    none
%
% Output arguments:
%    none

% ----------------------------------------------------------------------
% File.....: Plot_traj.m
% Date.....: 03-Mar-2015
% Version..: 1.0 - Last update by G.Giorgi, 11-Mar-2015
% Author...: G. Giorgi
%            Institute for Communications and Navigation
%            Technische Universität München
% ----------------------------------------------------------------------



%% Plot
    %h_traj(count) = plot(Agent_trajectory(1),Agent_trajectory(2),'redx');    
    
    % Set 2D rotation matrix (based on current heading)
    triangle_rot = [cos(Agent_trajectory(7)),-sin(Agent_trajectory(7));sin(Agent_trajectory(7)),cos(Agent_trajectory(7))]; 
    % Set scaling of triangle
    triangle_sc=5; 
    % Compute x-y coordinates of the three corners of a triangle (pointing
    % in the direction of the movement)
    tr_triangle=triangle_sc.*triangle_rot*triangle_pt;
    % Plot oriented triangle
    h_traj(count) = patch(tr_triangle(1,:)+Agent_trajectory(1),tr_triangle(2,:)+Agent_trajectory(2),'black','FaceColor',0.9*[1 1 1],'EdgeColor',[0 0 0]);                        
    
    % Set 2D rotation matrix (based on current heading) for FOV
    fov_rot = [cos(Agent_trajectory(7)),-sin(Agent_trajectory(7));sin(Agent_trajectory(7)),cos(Agent_trajectory(7))]; 
    % Compute x-y coordinates of the points used to define the conic FOV (pointing
    % in the direction of the movement)
    tr_fov=fov_rot*cone_pt;
    % Plot oriented conic FOV 
    h_camfov(count) = patch(tr_fov(1,:)+Agent_trajectory(1),tr_fov(2,:)+Agent_trajectory(2),'black','FaceColor',0.9*[1 1 1],'EdgeColor',[1 1 1],...
                            'EdgeAlpha',0.5,'FaceAlpha',0.5);                        
    
    % Mark current observed features
    if ~isnan(idx_ex_loc)
        h_obs_feat(count) = plot(loc_features(idx_ex_loc,1),loc_features(idx_ex_loc,2),'redo');
    else
        h_obs_feat(count) = NaN;
    end
    
    
    if sel_type_traj == 1       % Square trajectory
        % Set 2D rotation matrix (based on estimated heading)
        triangle_rot_KFstate = [cos(Agent_trajectory_KFstate(3)),-sin(Agent_trajectory_KFstate(3));sin(Agent_trajectory_KFstate(3)),cos(Agent_trajectory_KFstate(3))];
        % Scale up this triangle
        triangle_sc = triangle_sc*2;
        % Compute x-y coordinates of the three corners of a triangle (pointing
        % in the direction of the movement)
        tr_triangle_KFstate=triangle_sc.*triangle_rot_KFstate*triangle_pt;
        % Plot computed location (KF)
        h_traj_KFstate(count) = patch(tr_triangle_KFstate(1,:)+Agent_trajectory_KFstate(1),tr_triangle_KFstate(2,:)+Agent_trajectory_KFstate(2),'black','EdgeColor',[1 0 1],'FaceAlpha',0);%,'FaceColor',0.5*[1 1 1]);
        % Plot 95% confidence ellipse for estimated trajectory (x-y)
        iQaux = inv(Sigma_state(1:2,1:2));
        cc=ncx2inv(0.95,map_dim,0); % Value of variable at 95% probability
        [Xell_traj,Yell_traj] = DrawEllipse(iQaux(1,1),iQaux(1,2),iQaux(2,2),0,0,-cc,Agent_trajectory_KFstate(1:2));
        h_traj_ell(count) = plot(Xell_traj,Yell_traj);
    
    elseif sel_type_traj == 2   % Circular trajectory
        % Set 2D rotation matrix (based on estimated heading)
        triangle_rot_KFstate = [cos(Agent_trajectory_KFstate(1)),-sin(Agent_trajectory_KFstate(1));sin(Agent_trajectory_KFstate(1)),cos(Agent_trajectory_KFstate(1))];
        % Scale up this triangle
        triangle_sc = triangle_sc*2;
        % Compute x-y coordinates of the three corners of a triangle (pointing
        % in the direction of the movement)
        tr_triangle_KFstate=triangle_sc.*triangle_rot_KFstate*triangle_pt;
        % Plot computed location (KF)
        h_traj_KFstate(count) = patch(tr_triangle_KFstate(1,:)+ref_traj(1)-ext_traj*sin(Agent_trajectory_KFstate(1)),...
                                      tr_triangle_KFstate(2,:)+ref_traj(2)+ext_traj*cos(Agent_trajectory_KFstate(1)),...
                                      'black','EdgeColor',[1 0 1],'FaceAlpha',0);%,'FaceColor',0.5*[1 1 1]);
        % Plot confidence interval for estimated trajectory (theta, visualized as circle around estimated position)
        %iQaux = pinv([ext_traj*cos(Agent_trajectory_KFstate(1));ext_traj*sin(Agent_trajectory_KFstate(1))]*Sigma_state*[ext_traj*cos(Agent_trajectory_KFstate(1));ext_traj*sin(Agent_trajectory_KFstate(1))]');
        %iQaux = inv(Sigma_state);
        %cc=ncx2inv(0.95,1,0); % Value of variable at 95% probability
        %[Xell_traj,Yell_traj] = DrawEllipse(iQaux(1,1),iQaux(1,2),iQaux(2,2),0,0,-cc,[ref_traj(1)-ext_traj*sin(Agent_trajectory_KFstate(1));ref_traj(2)+ext_traj*cos(Agent_trajectory_KFstate(1))]);
        ang=0:0.01:2*pi;
        xp=sqrt(Sigma_state(1))*cos(ang);
        yp=sqrt(Sigma_state(1))*sin(ang);
        h_traj_ell(count) = plot(ref_traj(1)-ext_traj*sin(Agent_trajectory_KFstate(1))+xp,ref_traj(2)+ext_traj*cos(Agent_trajectory_KFstate(1))+yp);
        %[Xell_traj,Yell_traj] = DrawEllipse(iQaux,iQaux,iQaux,0,0,-cc,[ref_traj(1)-ext_traj*sin(Agent_trajectory_KFstate(1));ref_traj(2)+ext_traj*cos(Agent_trajectory_KFstate(1))]);
        %h_traj_ell(count) = plot(Xell_traj,Yell_traj);
        
    else
        error('Trajectory not correctly plot. Check Plot_traj.m file');
    end
    
    
    
    
    
pause    
if vis_pers_traj==0
    delete(h_traj(count));              % Only plot actual position
    delete(h_camfov(count));            % .. and actual fov
    delete(h_traj_KFstate(count));      % Only plot actual estimated position
    delete(h_traj_ell(count));          % .. and associated ellipse
    
elseif vis_pers_traj==1
    delete(h_camfov(count));        % Delete past fovs
    delete(h_traj(count));          % Delete past true position
    delete(h_traj_KFstate(count));  % Delete past estimated position
    delete(h_traj_ell(count));      % .. and associated ellipse
    
    % Plot history of trajectory (gray: true / red: estimated)
    if sel_type_traj == 1       % Square trajectory
        h_traj(count) = plot(Agent_trajectory(1),Agent_trajectory(2),'Marker','x','MarkerEdgeColor',0.9*[1 1 1],'MarkerFaceColor',0.9*[1 1 1]);
        h_traj_KFstate(count) = plot(Agent_trajectory_KFstate(1),Agent_trajectory_KFstate(2),'Marker','o','MarkerEdgeColor',0.9*[1 0 0]);%,'MarkerFaceColor',0.9*[1 0 0]);
    elseif sel_type_traj == 2   % Circular trajectory
        h_traj(count) = plot(ref_traj(1)-ext_traj*sin(Agent_trajectory_KFstate(1)),ref_traj(2)+ext_traj*cos(Agent_trajectory_KFstate(1)),...
                             'Marker','x','MarkerEdgeColor',0.9*[1 1 1],'MarkerFaceColor',0.9*[1 1 1]);
        h_traj_KFstate(count) = plot(ref_traj(1)-ext_traj*sin(Agent_trajectory_KFstate(1)),ref_traj(2)+ext_traj*cos(Agent_trajectory_KFstate(1)),...
                                     'Marker','o','MarkerEdgeColor',0.9*[1 0 0]);%,'MarkerFaceColor',0.9*[1 0 0]);
    else
        error('Trajectory not correctly plot. Check Plot_traj.m file');
    end
end


if ~isnan(h_obs_feat(count))
    delete(h_obs_feat(count));  % Delete past marks (to observed features)
end