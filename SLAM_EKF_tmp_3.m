
    % SLAM 1 
    
    %% Prediction
    Agent_trajectory_KFpred = Agent_trajectory_KFstate - dt*dyn_traj/ext_traj;
    % Predicted covariance
    Sigma_state_pred = Sigma_state + Sigma_process;
    
    %% Update following observations
    
    % If there is no observation available:
    if  isnan(idx_ex_loc)
        Agent_trajectory_KFstate = Agent_trajectory_KFpred;
        Sigma_state=Sigma_state_pred;
    else % otherwise
        % Collect observations
        Obs         = [];
        Pred_obs    = [];
        H           = [];
        % Rotation of local coordinate system
        Rot_ = [cos(Agent_trajectory_KFpred(1)),sin(Agent_trajectory_KFpred(1));-sin(Agent_trajectory_KFpred(1)),cos(Agent_trajectory_KFpred(1))]; 
        derRot = [-sin(Agent_trajectory_KFpred(1)),cos(Agent_trajectory_KFpred(1));-cos(Agent_trajectory_KFpred(1)),-sin(Agent_trajectory_KFpred(1))];
        for i_obs=1:1:length(idx_ex_loc)
            Obs = [Obs ; loc_features_im(i_obs,1:map_dim)'];
            Pred_obs = [Pred_obs ; Rot_*( loc_features(idx_ex_loc(i_obs),1:map_dim)'- [ref_traj(1)-ext_traj*sin(Agent_trajectory_KFpred(1));ref_traj(2)+ext_traj*cos(Agent_trajectory_KFpred(1))] )];
            % Build Jacobian
            H = [H ; derRot*( loc_features(idx_ex_loc(i_obs),1:map_dim)'- [ref_traj(1)-ext_traj*sin(Agent_trajectory_KFpred(1));ref_traj(2)+ext_traj*cos(Agent_trajectory_KFpred(1))] )+...
                     Rot_*[ext_traj*cos(Agent_trajectory_KFpred(1));ext_traj*sin(Agent_trajectory_KFpred(1))] ];
        end
        
        % Kalman gain
        K = Sigma_state_pred*H'*inv( H*Sigma_state_pred*H'+kron(eye(length(idx_ex_loc)),Sigma_obs) );
        
        % Update state
        Agent_trajectory_KFstate = Agent_trajectory_KFpred + K*( Obs - Pred_obs );
        Sigma_state = (eye(size(K,1))-K*H)*Sigma_state_pred;
        
    end
    
    % Adjust angle so it belongs to [0,2*pi)
    if Agent_trajectory_KFstate(1) < 0
        Agent_trajectory_KFstate(1)=2*pi+Agent_trajectory_KFstate(1);
    end
    
    % Compute estimation error
    Est_error_traj=[Est_error_traj ; abs(Agent_trajectory_KFstate(1)-Agent_trajectory(7))];
    
    
    



