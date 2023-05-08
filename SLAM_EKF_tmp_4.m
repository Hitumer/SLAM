
    % SLAM 2
    
    %% Prediction
    
    % Number of feature points in the current map
    N_map = 0.5*(size(Agent_trajectory_KFstate,1)-1);
    
    Agent_trajectory_KFpred = Agent_trajectory_KFstate - dt*dyn_traj/ext_traj*[1;zeros(N_map*2,1)];
    % Predicted covariance
    Sigma_state_pred = Sigma_state + [ Sigma_process , zeros(1,2*N_map) ; zeros(2*N_map,1)  ,  zeros(2*N_map,2*N_map) ];
    
    
    %% Update following observations
    Pos_pred = [ref_traj(1)-ext_traj*sin(Agent_trajectory_KFpred(1));...
                ref_traj(2)+ext_traj*cos(Agent_trajectory_KFpred(1))];
    
    % If there is no observation available:
    if  isnan(idx_ex_loc)
        Agent_trajectory_KFstate = Agent_trajectory_KFpred;
        Sigma_state=Sigma_state_pred;
        
    else % otherwise
        ID_im_loc = [];
        ID_old_obs = [];
        ID_new_obs = [];
        
        % ID of currently observed features (idx_ex_loc is the index!)
        ID_im_loc = loc_features_im(:,3);
        
        % Find ID to previously observed features in the stored map
        ID_old_obs = intersect(ID_map_loc,ID_im_loc);
        
        if ~isempty(ID_old_obs) % Update with re-observed features
            % Collect observations
            Obs         = [];
            Pred_obs    = [];
            H           = [];
            % Rotation of local coordinate system
            Rot_ = [cos(Agent_trajectory_KFpred(1)),sin(Agent_trajectory_KFpred(1));-sin(Agent_trajectory_KFpred(1)),cos(Agent_trajectory_KFpred(1))];
            % Derivative:
            derRot = [-sin(Agent_trajectory_KFpred(1)),cos(Agent_trajectory_KFpred(1));-cos(Agent_trajectory_KFpred(1)),-sin(Agent_trajectory_KFpred(1))];
            
            for i_obs=1:1:length(ID_old_obs)
                
                % Find index to previously observed features in map being built (in vector of imaged feature)
                idx_obs = find( ID_old_obs(i_obs) == loc_features_im(:,3) );
                
                % Find index to previously observed features in map being built (in stored map matrix)
                idx_map = find( ID_old_obs(i_obs) == loc_features_map(:,3) );
                
                % Build observations
                Obs = [Obs ; loc_features_im(idx_obs,1:map_dim)'];
                Pred_obs = [Pred_obs ; Rot_*( loc_features_map(idx_map,1:map_dim)'- Pos_pred )];
                
                % Build Jacobian
                H = [H ; derRot*( loc_features_map(idx_map,1:map_dim)'-Pos_pred )+...
                         Rot_*[ext_traj*cos(Agent_trajectory_KFpred(1));ext_traj*sin(Agent_trajectory_KFpred(1))] , zeros(2,2*(idx_map-1)) , Rot_ , zeros(2,2*(N_map-idx_map)) ];
                
            end
            % Kalman gain
            K = Sigma_state_pred*H'*inv( H*Sigma_state_pred*H'+kron(eye(length(ID_old_obs)),Sigma_obs) );
            % Update state
            %( Map-Obs - Ck*Agent_trajectory_KFpred )
            Agent_trajectory_KFstate = Agent_trajectory_KFpred + K*( Obs - Pred_obs );
            %Sigma_state = (eye(size(K,1))-K*Ck)*Sigma_state_pred;
            Sigma_state = (eye(size(K,1))-K*H)*Sigma_state_pred; 
            
            % Update local map
            for i = 1:1:N_map
                loc_features_map(i,1:map_dim) = Agent_trajectory_KFstate(2+map_dim*(i-1):2+(map_dim-1)+map_dim*(i-1));
            end
        
        else
            % If no re-observed features are present, then propagate predicted position
            Agent_trajectory_KFstate = Agent_trajectory_KFpred;
            Sigma_state=Sigma_state_pred;
        end
        
        % Find new observations
        ID_new_obs = setdiff(ID_im_loc,ID_map_loc);
        Pos_est  = [ref_traj(1)-ext_traj*sin(Agent_trajectory_KFstate(1));...
                    ref_traj(2)+ext_traj*cos(Agent_trajectory_KFstate(1))];
            
        if ~isempty(ID_new_obs) % If there are observations, update indexes and augment state vector w/ covariance matrix
            
            
            % Reconstruct feature point coordinates and update indexes:
              % Rotation of local coordinate system
              Rot_ = [cos(Agent_trajectory_KFstate(1)),sin(Agent_trajectory_KFstate(1));-sin(Agent_trajectory_KFstate(1)),cos(Agent_trajectory_KFstate(1))];
              Jobs = [];
              N_newFeat = length(ID_new_obs); % Number of new features
              
              for i_obs=1:1:N_newFeat
              
                  % Update with ID of new features
                  idx_im = find( ID_new_obs(i_obs) == loc_features_im(:,3)); % Index to image feature
                  ID_map_loc = [ID_map_loc loc_features_im(idx_im,3)'];
                  
                  % Find index to currently observed features (new among all observed) 
                  %idxx=find( idx_new_obs(i_obs) == idx_ex_loc );
                  
                  % Recontruct based on current positon
                  rec_Feat  = (Rot_'*loc_features_im(idx_im,1:map_dim)' + Pos_est );
                  
                  % Update map of feature localizations features 
                  if isempty(loc_features_map)
                      loc_features_map = [rec_Feat' , loc_features_im(idx_im,3) ];
                  else
                      loc_features_map = [loc_features_map ; rec_Feat' , loc_features_im(idx_im,3) ];
                  end
                  
                  % Update state vector (append new features)
                  Agent_trajectory_KFstate =[ Agent_trajectory_KFstate ; rec_Feat];
                  
                  % Build Jacobian
                  derRot = [-sin(Agent_trajectory_KFpred(1)),cos(Agent_trajectory_KFpred(1));-cos(Agent_trajectory_KFpred(1)),-sin(Agent_trajectory_KFpred(1))];
                  Jobs = [Jobs ; derRot'*loc_features_im(idx_im,1:map_dim)'-[ext_traj*cos(Agent_trajectory_KFpred(1));ext_traj*sin(Agent_trajectory_KFpred(1))],...
                          zeros(2,2*N_map) ,  zeros(2,2*(i_obs-1)) , Rot_' , zeros(2,2*(N_newFeat-i_obs)) ];
                  
              end
              
              Jobs_ = [eye(1+2*N_map) zeros(1+2*N_map,2*N_newFeat) ; Jobs];
              Sigma_state = Jobs_*[ Sigma_state , zeros(1+2*N_map,2*N_newFeat) ; zeros(2*N_newFeat,1+2*N_map) , kron(eye(N_newFeat),Sigma_obs) ]*Jobs_';
            
        end
        
        
        Est_error_traj=[Est_error_traj ; abs(Agent_trajectory_KFstate(1)-Agent_trajectory(7))];

        ID_ = loc_features_map(:,3);
        aux=0;
        for j=1:1:length(ID_)
            idxx = find(ID_(j)==loc_features(:,3));
            aux=aux+norm( loc_features_map(j,1:2)-loc_features(idxx,1:2) );
        end
        Est_error_map=[Est_error_map ; aux ];
    end
%pause
    
    % Adjust angle so it belongs to [0,2*pi)
    if Agent_trajectory_KFstate(1) < 0
        Agent_trajectory_KFstate(1)=2*pi+Agent_trajectory_KFstate(1);
    end
    





