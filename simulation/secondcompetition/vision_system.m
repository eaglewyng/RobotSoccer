% Models vision system
% The output has the following format
%   out = [robot1; robot2; ... robotN], where
%   roboti = [ball; own_team; opponent; markers], where 
%   ball = [range; bearing]
%   own_team = [range1; bearing1; range2; bearing2; ...]
%     etc.
%   For each robot, the range and bearing to itself is 0, 0
%   When the range or bearing is outside the camera field of view, the
%   value is P.camera_out_of_range.
%
% Modified: 
%        2/11/2014 - R. Beard
%        3/5/2015 - R. Beard (fixed numerous bugs in original code)   
%

function vision_out=vision_system(uu,P)
  NN = 0;
  ownteam = reshape(uu(1+NN:3*P.num_robots+NN),3,P.num_robots);
  NN = NN + 3*P.num_robots;
  opponent = reshape(uu(1+NN:3*P.num_robots+NN),3,P.num_robots);
  NN = NN + 3*P.num_robots;
  ball = uu(1+NN:2+NN);
  NN = NN + 2;
    
  %%%%% WARNING %%%%
  % THIS CODE HAS NOT BEEN TESTED AND MAY HAVE BUGS
  %
  
  vision_out = [];
  % construct camera output for each robot on ownteam
  for i=1:P.num_robots,
      
      % heading vector of robot
      n = [cos(ownteam(3,i)); sin(ownteam(3,i))];
      
      % range and bearing to ball
      r = ball-ownteam(1:2,i);
      [range_to_ball, bearing_to_ball] = compute_range_bearing(r,n,P);
      vision_out = [vision_out; range_to_ball; bearing_to_ball];
      
      % range and bearing to ownteam robots
      for j = 1:P.num_robots,
          % set range and bearing to self to zero
          if i==j,
              range_to_ownteam = 0;
              bearing_to_ownteam = 0;
          else
              r = ownteam(1:2,j)-ownteam(1:2,i);
              [range_to_ownteam, bearing_to_ownteam] = compute_range_bearing(r,n,P);
          end
          vision_out = [vision_out; range_to_ownteam; bearing_to_ownteam];
      end
      
      % range and bearing to opponent robots
      for j = 1:P.num_robots,
          r = opponent(1:2,j)-ownteam(1:2,i);
          [range_to_opponent, bearing_to_opponent] = compute_range_bearing(r,n,P);
          vision_out = [vision_out; range_to_opponent; bearing_to_opponent];        
      end
      
      % range and bearing to markers
      for j=1:P.num_markers,
          r = P.marker(:,j)-ownteam(1:2,i);
          [range_to_marker, bearing_to_marker] = compute_range_bearing(r,n,P);
          vision_out = [vision_out; range_to_marker; bearing_to_marker];        
      end
   end

end

%---------------------------------------------------------
% compute range and bearing, checking camera constraints
function [range,bearing] = compute_range_bearing(r,n,P)
    if n'*r<=0, % object is behind robot
        range   = P.camera_out_of_range;
        bearing = P.camera_out_of_range;
    else % object is in front of robot
        tmp = r/norm(r);
        bearing = asin(n(1)*tmp(2)-n(2)*tmp(1));
        if ~isreal(bearing) || (abs(bearing)>P.camera_fov/2),
            bearing = P.camera_out_of_range;
            range   = P.camera_out_of_range;
        else
            range = norm(r);
            if (range<P.camera_min_range) || (range>P.camera_max_range),
                range = P.camera_out_of_range;
            end
        end
    end
end


  

  