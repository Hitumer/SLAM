function [idx] = Extract_idx_features(Cam_trajectory,loc_features,cam_fov,cam_dov,map_dim,num_features);

%%  Extract_idx_features : extract features currently whithin the FOV of camera (index to feature location matrix)
%
% This function returns the index to the currently observed features (i.e., whithin the camera FOV) in
% the map
%
% Syntax:
%    [Cam_trajectory] = Compute_cam_trajectory(Agent_trajectory,Cam_trajectory,map_dim,cam_lock)
%
% Input arguments:
%    Cam_trajectory     - Vector of camera dynamic parameters
%    loc_features       - Matrix of feature locations across map
%    cam_fov            - Camera field of view (FOV)
%    cam_dov            - Camera depth of view
%    map_dim            - Map dimensionality (see Init.m)
%    num_features       - Number of features
%
% Output arguments:
%    idx                - Index to observed features, in matrix
%                          loc_features (NaN if no features are in view)

% ----------------------------------------------------------------------
% File.....: Extract_idx_features.m
% Date.....: 10-Mar-2015
% Version..: 1.0 - Last update by G.Giorgi, 10-Mar-2015
% Author...: G. Giorgi
%            Institute for Communications and Navigation
%            Technische Universität München
% ----------------------------------------------------------------------


%% Algorithm

if map_dim==2               % IF 2D
    
    % Compute vectorial distance features-camera
    vec_dist=loc_features(:,1:map_dim)-repmat(Cam_trajectory(1:2),num_features,1);
    
    % Isolate features within dov along any direction from camera
    idx_a1 = find( abs(vec_dist(:,1)) < cam_dov );
    idx_a2 = find( abs(vec_dist(:,2)) < cam_dov );
    idx_a = intersect(idx_a1,idx_a2); 
    
    
    if isempty(idx_a)   % If no features in DOV, exit directly
        idx = NaN;
        
    else            % otherwise, check fov
        % Isolate remaining features within dov
        for i=1:1:length(idx_a)
            aux(i)=norm(vec_dist(idx_a(i),1:2));
        end
        idx_b = find( aux < cam_dov );
        idx_c = idx_a(idx_b);
        
        % Compute agent-feature vector heading
        feat_loc_heading = atan2(vec_dist(idx_c,2),vec_dist(idx_c,1)); % Heading of agent-to-feature vector
        
        % Correct atan2 output from [-pi,pi] to [0 2*pi]
        idx_neg = find(feat_loc_heading < 0 );
        feat_loc_heading(idx_neg) = 2*pi+feat_loc_heading(idx_neg);
        
        % Extract features within fov
        idx=NaN;
        cont_=0;
        for i=1:1:length(idx_c)
            if (abs( feat_loc_heading(i)-Cam_trajectory(3) ) < cam_fov/2) || (abs( feat_loc_heading(i)-Cam_trajectory(3) )  > 2*pi-cam_fov/2)
                cont_=cont_+1;
                idx(cont_)=idx_c(i);
            end
        end
    end
end




