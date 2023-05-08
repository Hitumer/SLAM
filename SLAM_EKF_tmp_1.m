
    % SLAM 1 
    
    %% Prediction
    Jk = [1 0 -dyn_traj*sin(Agent_trajectory_KFstate(3))*dt ;...
          0 1  dyn_traj*cos(Agent_trajectory_KFstate(3))*dt ;...
          0 0  1                                    ];
    Bk = [0,0,-pi/2]';
    %Bk = [dyn_traj*dt*(sin(Agent_trajectory_KFpred(3))-cos(Agent_trajectory_KFpred(3))),dyn_traj*dt*(cos(Agent_trajectory_KFpred(3))-sin(Agent_trajectory_KFpred(3))),-pi/2]';
%     if abs(Agent_trajectory(7)-Agent_trajectory_KFstate(3))>0.8 % Detect turn
%         uk = 1;
%     else
%         uk=0;
%     end
    % Predicted state
   
    Agent_trajectory_KFpred = [Agent_trajectory_KFstate(1)+dyn_traj*cos(Agent_trajectory_KFstate(3))*dt ;...
                               Agent_trajectory_KFstate(2)+dyn_traj*sin(Agent_trajectory_KFstate(3))*dt ;...
                               Agent_trajectory_KFstate(3) ] + Bk*uk;
    % Re-instate command to zero
    uk=0;
    % Predicted covariance
    Sigma_state_pred = Jk*Sigma_state*Jk' + Sigma_process;
    
    %% Update following observations
    
    % If there is no observation available:
    if  isnan(idx_ex_loc)
        Agent_trajectory_KFstate = Agent_trajectory_KFpred;
        Sigma_state=Sigma_state_pred;
    else % otherwise
        % Collect observations
        Obs         = [];
        Pred_obs    = [];
        Jobs        = [];
        % Rotation of local coordinate system
        Rot_ = [cos(Agent_trajectory_KFpred(3)),sin(Agent_trajectory_KFpred(3));-sin(Agent_trajectory_KFpred(3)),cos(Agent_trajectory_KFpred(3))]; 
        for i_obs=1:1:length(idx_ex_loc)
            Obs = [Obs ; loc_features_im(i_obs,1:map_dim)'];
            Pred_obs = [Pred_obs ; Rot_*( loc_features(idx_ex_loc(i_obs),1:map_dim)'- Agent_trajectory_KFpred(1:2) )];
            % Build Jacobian
            derRot = [-sin(Agent_trajectory_KFpred(3)),cos(Agent_trajectory_KFpred(3));-cos(Agent_trajectory_KFpred(3)),-sin(Agent_trajectory_KFpred(3))];
            Jobs = [Jobs ; -Rot_  , derRot*( loc_features(idx_ex_loc(i_obs),1:map_dim)'- Agent_trajectory_KFpred(1:2) ) ];
        end
        
        % Kalman gain
        K = Sigma_state_pred*Jobs'*inv( Jobs*Sigma_state_pred*Jobs'+kron(eye(length(idx_ex_loc)),Sigma_obs) );
        
        % Update state
        %( Map-Obs - Ck*Agent_trajectory_KFpred )
        Agent_trajectory_KFstate = Agent_trajectory_KFpred + K*( Obs - Pred_obs );
        %Sigma_state = (eye(size(K,1))-K*Ck)*Sigma_state_pred;
        Sigma_state = (eye(size(K,1))-K*Jobs)*Sigma_state_pred;
        Est_error_traj=[Est_error_traj ; norm(Agent_trajectory_KFstate(1:2)-Agent_trajectory(1:2))];
    end
    
    % Adjust angle so it belongs to [0,2*pi)
    if Agent_trajectory_KFstate(3) < 0
        Agent_trajectory_KFstate(3)=2*pi+Agent_trajectory_KFstate(3);
    end
    





