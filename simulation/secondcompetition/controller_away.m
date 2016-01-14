% main control code 
%
%
% Modified: 
%   2/11/2014 - R. Beard
%   2/18/2014 - R. Beard
%   2/24/2014 - R. Beard
%   3/5/2014  - R. Beard
%

function out=controller_away(uu,P)

    persistent v1
    persistent v2
    persistent reset_flag
    persistent old_score
    
    % put input data into appropriate data structures
    for i=1:P.num_robots,
        vision(i) = utility_process_vision_data(uu,i,P);
    end
    markers = reshape(uu((end-2-2*P.num_markers):(end-3)),2,P.num_markers);    
    score = [uu(end-2); uu(end-1)];
    t = uu(end);

    % initialize persistent variables
    if t==0,
        v1 = [0; 0; 0];
        v2 = [0; 0; 0];
        reset_flag = 1;
        old_score = score;
    end
    
    % if score changes, set reset flag
    if norm(score-old_score)~=0,
        reset_flag = 1;
        old_score = score;
    end
    
    % estimate the state of the world (using vision data from ith robot)
    state = utility_observer(vision, markers, v1, v2, reset_flag, P);
    
    
    % robot #1 positions itself behind ball and rushes the goal.
    %v1 = play_rush_goal(robot(:,1), ball, P);
    v1 = skill_follow_ball_on_line(state(1).self, state(1).ball(1:2), -P.field_width/3, P);
 
    % robot #2 stays on line, following the ball, facing the goal
    v2 = skill_follow_ball_on_line(state(2).self, state(2).ball(1:2), -2*P.field_width/3, P);

    
    
    % output velocity commands to robots
    v1 = utility_saturate_velocity(v1,P);
    v2 = utility_saturate_velocity(v2,P);
    
    % output velocities and position estimate/covariance of one object
    out = [v1; v2];
    
    % reset flag is zero for next iteration (only reset once)
    reset_flag = 0;
end

%-----------------------------------------
% play - rush goal
%   - go to position behind ball
%   - if ball is between robot and goal, go to goal
% NOTE:  This is a play because it is built on skills, and not control
% commands.  Skills are built on control commands.  A strategy would employ
% plays at a lower level.  For example, switching between offense and
% defense would be a strategy.
function v = play_rush_goal(robot, ball, P)
  
  % normal vector from ball to goal
  n = P.goal-ball;
  n = n/norm(n);
  % compute position 10cm behind ball, but aligned with goal.
  position = ball - 0.2*n;
    
  if norm(position-robot(1:2))<.21,
      v = skill_go_to_point(robot, P.goal, P);
  else
      v = skill_go_to_point(robot, position, P);
  end

end

%-----------------------------------------
% skill - follow ball on line
%   follows the y-position of the ball, while maintaining x-position at
%   x_pos.  Angle always faces the goal.

function v=skill_follow_ball_on_line(robot, ball, x_pos, P)

    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-x_pos);
    
    % control y position to match the ball's y-position
    vy = -P.control_k_vy*(robot(2)-ball(2));

    % control angle to -pi/2
    theta_d = atan2(P.goal(2)-robot(2), P.goal(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];
end

%-----------------------------------------
% skill - go to point
%   follows the y-position of the ball, while maintaining x-position at
%   x_pos.  Angle always faces the goal.

function v=skill_go_to_point(robot, point, P)

    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-point(1));
    
    % control y position to match the ball's y-position
    vy = -P.control_k_vy*(robot(2)-point(2));

    % control angle to -pi/2
    theta_d = atan2(P.goal(2)-robot(2), P.goal(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];
end


%------------------------------------------
% utility - saturate_velocity
% 	saturate the commanded velocity 
%
function v = utility_saturate_velocity(v,P)
    if v(1) >  P.robot_max_vx,    v(1) =  P.robot_max_vx;    end
    if v(1) < -P.robot_max_vx,    v(1) = -P.robot_max_vx;    end
    if v(2) >  P.robot_max_vy,    v(2) =  P.robot_max_vy;    end
    if v(2) < -P.robot_max_vy,    v(2) = -P.robot_max_vy;    end
    if v(3) >  P.robot_max_omega, v(3) =  P.robot_max_omega; end
    if v(3) < -P.robot_max_omega, v(3) = -P.robot_max_omega; end
end

%------------------------------------------
% utility - process vision data
% 	extract vision data out of the input
%
function vision = utility_process_vision_data(in, i, P)
    NN = (2+2*(P.num_robots)+2*P.num_robots+2*P.num_markers)*(i-1);
    vision.ball     = in(1+NN:2+NN);
    NN = NN + 2;
    vision.ownteam = [];
    for j=1:P.num_robots,
        if i==j,
            NN = NN + 2;
        else
            vision.ownteam  = [vision.ownteam, in(1+NN:2+NN)];  
            NN = NN + 2;
        end
    end
    vision.opponent = [];
    for j=1:P.num_robots,
        vision.opponent = [vision.opponent, in(1+NN:2+NN)];  
        NN = NN + 2;
    end
    vision.marker   = reshape(in(1+NN:2*P.num_markers+NN),2,P.num_markers); 
    %NN = NN + 2*P.num_markers;
end
 
%------------------------------------------
% utility observer
% 	estimate position of 
%       - robot
%       - ownteam
%       - opponent
%       - ball
%
% We need a different observer function for each robot so that the
% persistent variables don't get mixed up.
%
function state = utility_observer(vision, markers, v1, v2, reset_flag, P)
    
    persistent state_
    
    % observer gains (tune these)
    % we put the tunable gains here to make each team's control files 
    % independent of param.m
    S0_self = 10*diag([P.field_length/12; P.field_width/12; pi/2]);
    Q_self = 10000*eye(3);
    R_range = .1;
    R_bearing = 1*pi/180;
    S0_ball = diag([1,1,1,1,1]);
    Q_ball = diag([1,1,10,10,.1]);


    if reset_flag==1,
        state_(1).self = P.home_team_initial_configurations(1:3);
        state_(1).S_self = S0_self;
        state_(2).self = P.home_team_initial_configurations(4:6);
        state_(2).S_self = S0_self;
        state_(1).ball = [0; 0; 0; 0; .3];  % position and velocity and friction coefficient
        state_(1).S_ball = S0_ball;
        state_(2).ball = [0; 0; 0; 0; .3];  % position and velocity and friction coefficient
        state_(2).S_ball = S0_ball;
    end
   
    %----------------------
    % estimate self position and heading
    [state_(1).self, state_(1).S_self] = utility_observer_self_update(state_(1).self,state_(1).S_self,vision(1),markers,v1,Q_self,R_range,R_bearing,P);
    [state_(2).self, state_(2).S_self] = utility_observer_self_update(state_(2).self,state_(2).S_self,vision(2),markers,v2,Q_self,R_range,R_bearing,P);

    %----------------------
    % estimate ball position and velocity
    [state_(1).ball, state_(1).S_ball] = utility_observer_ball_update(state_(1).ball,state_(1).S_ball,vision(1),state_(1).self,Q_ball,R_range,R_bearing,P);
    [state_(2).ball, state_(2).S_ball] = utility_observer_ball_update(state_(2).ball,state_(2).S_ball,vision(2),state_(2).self,Q_ball,R_range,R_bearing,P);

    state = state_;
end
    
    
%------------------------------------------
% utility_observer_self_update
%   given vision measurement, update estimate of self
%
function [xhat, S] = utility_observer_self_update(xhat, S, vision, markers, velocity, Q_self, R_range, R_bearing, P)

    % estimate between measurements
    N = 10;
    for i=1:N,
        f = velocity;
        xhat = xhat + (P.control_sample_rate/N)*f;
        S = S + (P.control_sample_rate/N)*(Q_self);
    end
 
    % measurement updates
    for i=1:P.num_markers,
        % range measurement
        if vision.marker(1,i)~=P.camera_out_of_range,
            rho = markers(:,i)-xhat(1:2);
            Rho = norm(rho);
            h = Rho;
            C = [-rho(1), -rho(2), 0]/Rho;
            L = S*C'/(R_range+C*S*C');
            S = (eye(3)-L*C)*S;
            xhat = xhat + L*(vision.marker(1,i)-h);
        end
        % bearing measurement
        if vision.marker(2,i)~=P.camera_out_of_range,
            rho = markers(:,i)-xhat(1:2);
            Rho = norm(rho);
            phi = xhat(3);
            h = asin((rho(2)*cos(phi)-rho(1)*sin(phi))/Rho);
            C = sign(rho(1)*cos(phi)+rho(2)*sin(phi))...
                *[rho(2)/(Rho^2), -rho(1)/(Rho^2), -1];
            L = S*C'/(R_bearing+C*S*C');
            S = (eye(3)-L*C)*S;
            xhat = xhat + L*(vision.marker(2,i)-h);
        end
    end   
end

 %------------------------------------------
% utility_observer_ball_update
%   given vision measurement, update estimate of ball position, velocity,
%   and friction coefficient
%
function [xhat, S] = utility_observer_ball_update(xhat, S, vision, robot, Q_ball, R_range, R_bearing, P)

    % estimate between measurements
    N = 10;
    for i=1:N,
        % right hand side of differential equation for ball
        % doesn't include bounces off wall
        f = [...
            xhat(3:4);...
            -xhat(5)*xhat(3:4);...
            0;...
            ];
        xhat = xhat + (P.control_sample_rate/N)*f;
        A = [...
            zeros(2,2), eye(2), zeros(2,1);...
            zeros(2,2), -xhat(5)*eye(2), -xhat(3:4);...
            zeros(1,2), zeros(1,2), 0;...
            ];
        S = S + (P.control_sample_rate/N)*(A*S+S*A'+Q_ball);
    end
 
    % measurement updates
    % range measurement
    if vision.ball(1)~=P.camera_out_of_range,
        rho = xhat(1:2)-robot(1:2);
        Rho = norm(rho);
        h = Rho;
        C = [rho(1), rho(2), 0, 0, 0]/Rho;
        L = S*C'/(R_range+C*S*C');
        S = (eye(5)-L*C)*S;
        xhat = xhat + L*(vision.ball(1)-h);
    end
    % bearing measurement
    if vision.ball(2)~=P.camera_out_of_range,
        rho = xhat(1:2)-robot(1:2);
        Rho = norm(rho);
        phi = robot(3);
        h = asin((rho(2)*cos(phi)-rho(1)*sin(phi))/Rho);
        C = sign(rho(1)*cos(phi)+rho(2)*sin(phi))...
            *[-rho(2)/(Rho^2), rho(1)/(Rho^2), 0, 0, 0];
        L = S*C'/(R_bearing+C*S*C');
        S = (eye(5)-L*C)*S;
        xhat = xhat + L*(vision.ball(2)-h);
    end
 end


  