function [loc_features_im] = Image_features(Cam_trajectory,loc_features,map_dim,cam_type,sigmas)

%%  Image_features : returns the feature coordinate in a local (camera-centered) frame 
%
% This function simulates the imaging process of the camera. It takes the
% observed, actual coordinates of the featuresin view and returns their
% coordinates as reconstructed by the camera (NOTA: MONOCULAR TO
% INVESTIGATE, POSSIBLY REFORMULATE WITH ACTUAL IMAGING PROCESS, with camera matrix and all)
% Gaussian noise is added TODO
%
% Syntax:
%    [loc_features_im] = Image_features(Cam_trajectory,loc_features,cam_type)
%
% Input arguments:
%    Cam_trajectory     - Vector of camera dynamic parameters
%    loc_features       - Matrix of feature locations (only those in view)
%    map_dim            - Map dimensionality (see Init.m)
%    cam_type           - Camera type (stereo/mono, see Init.m)
%    sigmas             - Noise nominal values (see Init.m)
%
% Output arguments:
%    loc_features_im    - Local coordinates of observed features
% ----------------------------------------------------------------------
% File.....: Image_features.m
% Date.....: 12-Mar-2015
% Version..: 1.0 - Last update by G.Giorgi, 12-Mar-2015
% Author...: G. Giorgi
%            Institute for Communications and Navigation
%            Technische Universität München
% ----------------------------------------------------------------------


%% Algorithm

obs_features = size(loc_features,1);  % Number of features in view

if map_dim==2               % IF 2D
    
    if cam_type==2              % IF Stero
        
        % Compute vectorial distance features-camera
        loc_features_im=loc_features(:,1:map_dim)-repmat(Cam_trajectory(1:2),obs_features,1);
        
        % Rotate vector so to refer to current camera heading
        Rot = [cos(Cam_trajectory(3)),sin(Cam_trajectory(3));-sin(Cam_trajectory(3)),cos(Cam_trajectory(3))]; 
        mean_loc_features_im=loc_features_im*Rot';
        %loc_features_im=loc_features_im';
        
        % Add (Gaussian) noise
        for i=1:1:obs_features
            % Compute range feature-to-camera
            range(i)     = norm(mean_loc_features_im(i,:));                      
            % Create 2D covaraince matrix (oblate in x-direction, to simulate depth higher uncertainty) 
            %Q_feat_im(1:2,1:2,i) = [range(i)*sigmas(1)^2                         sigmas(3)*sqrt(range(i))*sigmas(1)*sigmas(2) 
            Q_feat_im = [range(i)*sigmas(1)^2                         sigmas(3)*sqrt(range(i))*sigmas(1)*sigmas(2) 
                         sigmas(3)*sqrt(range(i))*sigmas(1)*sigmas(2) sigmas(2)^2                                  ];
            loc_features_im(i,:) = mvnrnd(mean_loc_features_im(i,:),Q_feat_im); 
            %loc_features_im(i,:) = mean_loc_features_im(i,:);
        end
        
    end
end




