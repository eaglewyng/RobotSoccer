% main control code - assumes full state knowledge
%
%
% Modified: 
%   2/11/2014 - R. Beard
%   2/18/2014 - R. Beard
%   2/24/2014 - R. Beard
%   1/4/2016  - R. Beard
%

% this first function catches simulink errors and displays the line number


function v_c=controller_home(uu,P)
    try
        v_c=controller_home_(uu,P);
    catch e
        msgString = getReport(e);
        fprintf(2,'\n%s\n',msgString);
        rethrow(e);
    end
end

% main control function
function v_c=controller_home_(uu,P)   
    %choose strategy
    
    totalGameTime = 90;
    
    %overall state bits
    persistent nearEndOfGame;
    persistent leadingBy2;
    persistent trailingBy2;
    persistent defaultStrategyFailed;
    persistent winning;
    
    if(isempty(nearEndOfGame))
        nearEndOfGame = 0;
    end
    if(isempty(leadingBy2))
        leadingBy2 = 0;
    end
    if(isempty(trailingBy2))
        trailingBy2 = 0;
    end
    if(isempty(defaultStrategyFailed))
        defaultStrategyFailed = 0;
    end
    if(isempty(winning))
        winning = 0;
    end
    
    
    % process inputs to function
    % robots - own team
    for i=1:P.num_robots,
        robot(:,i)   = uu(1+3*(i-1):3+3*(i-1));
    end
    NN = 3*P.num_robots;
    % robots - opponent
    for i=1:P.num_robots,
        opponent(:,i)   = uu(1+3*(i-1)+NN:3+3*(i-1)+NN);
    end
    NN = NN + 3*P.num_robots;
    % ball
    ball = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    % score: own team is score(1), opponent is score(2)
    score = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    % current time
    t      = uu(1+NN);
    
    if(t < 3)
        defaultStrategyFailed = 0;
    end
    
    %update persistent vars if needed
    if(t >= (totalGameTime - totalGameTime * .3))
        nearEndOfGame = 1;
    else
        nearEndOfGame = 0;
    end
    if(score(1) - score(2) >= 2)
        leadingBy2 = 1;
    else
        leadingBy2 = 0;
    end
    if(score(2) - score(1) >= 2)
        trailingBy2 = 1;
        defaultStrategyFailed = 1;
    else
        trailingBy2 = 0;
    end
    if(score(1) > score(2))
        winning = 1;
    else
        winning = 0;
    end
    
    %if we are near the end of the game and we are not winning,
    %take on an aggressive strategy. If we're wining, take a defensive
    %strategy.
    if(nearEndOfGame == 1)
        if(winning == 0)
            v_c = strategy_strong_offense(P, robot, ball);
        else
            %Go super defensive.
            v_c = strategy_strong_defense(P, robot, ball);
        end
        
    %if we're leading by 2, take a defensive strategy    
    elseif(leadingBy2==1)
        v_c = strategy_strong_defense(P, robot, ball);
    elseif(trailingBy2)
        v_c = strategy_strong_offense(P, robot, ball);
    elseif(defaultStrategyFailed == 1)
        %do a different strategy
        %TODO: evaluate strategies from last years' teams
        v_c = strategy_strong_offense(P, robot, ball);
    else
        v_c = strategy_intelligent(P, robot, ball);
    end
    
    
    
    
end

function v_c = strategy_strong_offense(P, robot, ball)
    defense = 0;
    offense = 1;
    playtype = offense;
    
    % If the ball gets behind 3/4 field then go on defense
    if(ball(1) < (3*P.field_length/12)) %P.field_width/8
        playtype = defense;
    end
    
    v1 = play_rush_goal(robot(:,1), ball, P);
    
    if(playtype == offense)
        v2 = skill_follow_ball_on_line(robot(:,2), ball, P.field_length/6, P);
    else
        v2 = play_rush_goal(robot(:,2), ball, P);
    end
    
    v1 = utility_saturate_velocity(v1,P);
    v2 = utility_saturate_velocity(v2,P);
    v_c = [v1; v2];
end


%-----------------------------------------
% play - rush goal
%   - go to position behind ball
%   - if ball is uubetween robot and goal, go to goal
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
      %v = wall_bounce_to_goal(robot, ball, P);
  else
      v = skill_go_to_point(robot, position, P);
  end

end

function v = wall_bounce_to_goal(robot, ball, P)

if(robot(2) < 0)
    
    x_hit_location = (robot(1) + P.goal(1))/2
    y_hit_location = -P.field_width/2;
    v = skill_go_to_point(robot, [x_hit_location, y_hit_location], P);
    
else
   x_hit_location = (robot(1) + P.goal(1))/2
   y_hit_location = P.field_width/2;
   v = skill_go_to_point(robot, [x_hit_location, y_hit_location], P);
    
end


end



function v_c=strategy_default(uu,P)
    % process inputs to function
    % robots - own team
    for i=1:P.num_robots,
        robot(:,i)   = uu(1+3*(i-1):3+3*(i-1));
    end
    NN = 3*P.num_robots;
    % robots - opponent
    for i=1:P.num_robots,
        opponent(:,i)   = uu(1+3*(i-1)+NN:3+3*(i-1)+NN);
    end
    NN = NN + 3*P.num_robots;
    % ball
    ball = [uu(1+NN); uu(2+NN)];
    NN = NN + 2;
    
    % robot #1 positions itself behind ball and rushes the goal.
    if(ball(1) < 0 )

        v1 = play_rush_goal(robot(:,1), ball, P);
        %v1 = skill_between_ball_and_goal(robot(:,1), ball, P);
    else
        v1 = play_rush_goal(robot(:,1), ball, P);
    end


    % robot #2 stays on line, following the ball, facing the goal
    if(ball(1) < 0)
        v2 = skill_guard_goal(robot(:,2), ball, -P.field_width + .05 , P);
    else
        v2 = skill_follow_ball_on_line(robot(:,2), ball, 0 , P);
    end


    
    % output velocity commands to robots
    v1 = utility_saturate_velocity(v1,P);
    v2 = utility_saturate_velocity(v2,P);
    v_c = [v1; v2];
end

function v_c=strategy_intelligent(P, robot, ball)
    persistent robotMode;

    

    %0 = Robot 1 on Offense, Robot 2 on Defense
    %1 = Robot 1 on Defense, Robot 2 on Offense
    %2 = Robot 1 on Offense, Robot 2 on Offense
    %3 = Robot 1 on Defense, Robot 2 on Defense
    
    robot1 = robot(:,1);
    distanceR1 = sqrt((robot1(1)-ball(1))^2 + (robot1(2) - ball(2))^2);
    
    robot2 = robot(:,2);
    distanceR2 = sqrt((robot2(1)-ball(1))^2 + (robot2(2) - ball(2))^2);

    
    if isempty(robotMode)
        robotMode = 0;
    end
    
    %should we change modes?
    if(distanceR1 < .20)
        robotMode = 0;
    elseif(distanceR2 < .20)
        robotMode = 1;
    end
    
    midOffset = .05;
    
    if (robotMode == 0)
    % robot #1 positions itself behind ball and rushes the goal.
        if(ball(1) < 0 )
            v1 = play_rush_goal(robot(:,1), ball, P);
            %v1 = skill_between_ball_and_goal(robot(:,1), ball, P);
        else
            v1 = play_rush_goal(robot(:,1), ball, P);
        end


        % robot #2 stays on line, following the ball, facing the goal
        if(ball(1) < 0)
            v2 = skill_guard_goal(robot(:,2), ball, -P.field_width + .05 , P);
        else
            v2 = skill_follow_ball_on_line(robot(:,2), ball, 0 + midOffset, P);
        end
    elseif(robotMode == 1)
        if(ball(1) < 0 )
            v2 = play_rush_goal(robot(:,2), ball, P);
            %v1 = skill_between_ball_and_goal(robot(:,1), ball, P);
        else
            v2 = play_rush_goal(robot(:,2), ball, P);
        end


        % robot #2 stays on line, following the ball, facing the goal
        if(ball(1) < 0)
            v1 = skill_guard_goal(robot(:,1), ball, -P.field_width + .05 , P);
        else
            v1 = skill_follow_ball_on_line(robot(:,1), ball, 0 + midOffset, P);
        end
    elseif(robotMode == 2)
        if(ball(1) < 0 )
            v1 = play_rush_goal(robot(:,1), ball, P);
            v2 = play_rush_goal(robot(:,2), ball, P);
            %v1 = skill_between_ball_and_goal(robot(:,1), ball, P);
        else
            v1 = play_rush_goal(robot(:,1), ball, P);
            v2 = play_rush_goal(robot(:,2), ball, P);
        end
    else
        v1 = skill_follow_ball_on_line(robot(:,1), ball, -P.field_length/4, P); 
        v2 = skill_guard_goal(robot(:,2), ball, -P.field_width + .05, P);
    end

    
    % output velocity commands to robots
    v1 = utility_saturate_velocity(v1,P);
    v2 = utility_saturate_velocity(v2,P);
    v_c = [v1; v2];
end

function v_c = strategy_puppyguard_goal(P, robot, ball)

    yoffset = .15;
    if(ball(2) <= 0)
        v1 = skill_guard_goal_secondary(robot(:,1), ball, -P.field_width + .05, yoffset, P);
        v2 = skill_guard_goal(robot(:,2), ball, -P.field_width + .05, P);
    else
        v2 = skill_guard_goal_secondary(robot(:,2), ball, -P.field_width + .05, yoffset, P);
        v1 = skill_guard_goal(robot(:,1), ball, -P.field_width + .05, P);
    end
    
    % output velocity commands to robots
    v1 = utility_saturate_velocity(v1,P);
    v2 = utility_saturate_velocity(v2,P);
    v_c = [v1; v2];
        
end

function v_c = strategy_debug()
    v1 = 0;
    v2 = 0;
    v_c = [v1; v2];
end

function v_c = strategy_strong_defense(P, robot, ball)

    v1 = skill_guard_goal(robot(:,1), ball, -P.field_width + .05, P);
    v2 = skill_between_ball_and_goal(robot(:,2), ball, P);

    % output velocity commands to robots
    v1 = utility_saturate_velocity(v1,P);
    v2 = utility_saturate_velocity(v2,P);
    v_c = [v1; v2];

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

function v=skill_guard_goal(robot, ball, x_pos, P)
    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-x_pos);

    
    % control y position to match the ball's y-position if it is within the
    % goal's bounds
    goalOffset = .10;
    
    topGoalY = goalOffset + P.goal_width / 2;
    bottomGoalY = -goalOffset + -1 * P.goal_width / 2;
    if ball(2) >= topGoalY,
        vy = -P.control_k_vy * (robot(2)-topGoalY);
    elseif ball(2) <= bottomGoalY,
        vy = -P.control_k_vy * (robot(2)-bottomGoalY);
    else
        vy = -P.control_k_vy*(robot(2)-ball(2));
    end
    
    % control angle to -pi/2
    theta_d = atan2(ball(2) - robot(2), ball(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];

end

function v=skill_guard_goal_secondary(robot, ball, x_pos, yoffset, P)
    % control x position to stay on current line
    vx = -P.control_k_vx*(robot(1)-x_pos);

    
    % control y position to match the ball's y-position if it is within the
    % goal's bounds
    goalOffset = .10;
    
    topGoalY = goalOffset + P.goal_width / 2;
    bottomGoalY = -goalOffset + -1 * P.goal_width / 2;
    if ball(2) >= topGoalY,
        vy = -P.control_k_vy * (robot(2)-topGoalY-yoffset);
    elseif ball(2) <= bottomGoalY,
        vy = -P.control_k_vy * (robot(2)-bottomGoalY + yoffset);
    elseif(ball(2) <= 0)
        vy = -P.control_k_vy*(robot(2)-ball(2)+yoffset);
    else
        vy = -P.control_k_vy*(robot(2)-ball(2)-yoffset);
    end
    
    % control angle to -pi/2
    theta_d = atan2(ball(2) - robot(2), ball(1)-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    
    v = [vx; vy; omega];
end


function v=skill_between_ball_and_goal(robot, ball, P)
    
    % control y position to match the ball's y-position if it is within the
    % goal's bounds
    point = [-P.field_length/2 + 2/3*(abs(P.field_length/2) - abs(ball(1))), 2/3*ball(2)];
    goalX = P.field_length/2;
    goalY = 0;
    
%     % control angle to -pi/2
    
%  
    leastX = -P.field_width + P.field_length / 10 + .05; 
    if (point(1) < leastX)
        vtemp = skill_go_to_point(robot, [leastX, point(2)], P);
    else
        vtemp = skill_go_to_point(robot, point, P);
    end
%     theta_d = atan2(ball(2), ball(1));
%     omega = -P.control_k_phi*(robot(3) - theta_d); 
    %theta_d = atan2(ball(2) - robot(2), ball(1)-robot(1));
    theta_d = atan2(goalY - robot(2), goalX-robot(1));
    omega = -P.control_k_phi*(robot(3) - theta_d); 
    v = [vtemp(1); vtemp(2); omega];
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


  