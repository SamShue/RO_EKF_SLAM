clc;                                %clear command window
clear all;
close all;                          %close all figures
rosshutdown                         %close current ros incase it is already initalized 

% PC Specific environment variables. Comment out when not needed.
% setenv('ROS_HOSTNAME', 'rahul-ThinkPad-S3-Yoga-14');
% setenv('ROS_IP', '192.168.1.104');

% Robot network variables
ipaddress = 'http://192.168.1.13:11311';         %define ipadress of turtlebot
setenv('ROS_MASTER_URI', ipaddress);
rosinit(ipaddress,'NodeHost','192.168.1.133')                  %initate ros using turtlebot IP

%odom = rossubscriber('/robot_pose_ekf/odom_combined');  %initialize a subscriber node to odomotry data
odom = rossubscriber('/odom');

% EKF Parameter Values
C = 0.2;    % Process Noise Constant
Rc = 1;   % Measurement Noise Constant

landmark_list=[]; %this is an input to the function and can be either empty or full of stuff
ekf_init = 0;
odomTS = 0;
laserTS = 0;
while(1)
    % Receive ROS Topics
    %======================================================================
    odomData = receive(odom);
    % End receive ROS topics
    %----------------------------------------------------------------------
    
    % Calculate Odometry
    %======================================================================
    % odometry data calculated here is the dead-reckoned position from
    % odometry readings in ROS.
    
    p = odomData.Pose.Pose;
    x_o = p.Position.X;
    y_o = p.Position.Y;
    
    quat = p.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = rad2deg(angles(1));
    
    rot_deg = 0;  % Turtlebot's heading may be in y direction
    % Rotate frame so heading is in x.
    pr = [cos(rot_deg) -sind(rot_deg); sind(rot_deg) cos(rot_deg)]*[x_o,y_o]';
    
    % store current position in terms of x, y, theta (degrees) as ROS sees
    odomPose = [pr(1),pr(2),theta];
    % End calculate odometery
    %----------------------------------------------------------------------
    
    % Estimate Robot's pose
    %======================================================================

    if(~exist('x'))
        oldOdomPose = odomPose;
        oldOdomData = odomData;
        % State Vector
        x = [0,0,0];
        % Covariance Matrix
        P = eye(length(x)).*0.1; 
        P(1,1) = 0.1; P(2,2) = 0.1; P(3,3) = 0.1;
        u = [0, 0];
    else
        % Get control vector (change in linear displacement and rotation)to
        % estimate current pose of the robot
        delta_D = sqrt((odomPose(1) - oldOdomPose(1))^2 + (odomPose(2) - oldOdomPose(2))^2);
        delta_Theta = rad2deg(angdiff(deg2rad(oldOdomPose(3)),deg2rad(odomPose(3))));
        u = [delta_D, delta_Theta];
        
        % Get noise covariance matrix for control signal
        W = [u(1)*cosd(x(3)) u(1)*sind(x(3)) u(2)]';
        Q = zeros(size(P));
        Q(1:3,1:3) = W*C*W';
        
        % Update position estimate
        [x,P] = RO_EKF_SLAM_Prediction(x,P,u,Q);
        % Record current odometry pose for next iteration
        oldOdomPose = odomPose;
        oldOdomData = odomData;
    end
    
    % Search for landmarks
    observed_LL = getLandmarkRSSI(landmark_list);

    % Apply measurement update in EKF if landmarks are observed
    if(~isempty(observed_LL))
        [numOfLandmarks ~] = size(observed_LL);
        for ii = 1:numOfLandmarks
            % Measurement vector
            z = [observed_LL(ii,1), observed_LL(ii,2)];
            % Measurement noise covariance matrix
            R = zeros(2,2); R(1,1) = observed_LL(ii,1)*Rc; R(2,2) = observed_LL(ii,2)*Rc;
            % Landmark index
            idx = observed_LL(ii,3);
        
            % Apply EKF measurement update
            [x,P] = RO_EKF_SLAM_Measurement(x,P,z,R,idx);
            updateLandmarkListRSSI(landmark_list,observed_LL);
        end
    end
    % End estimate robot's pose
    %----------------------------------------------------------------------
        
    % Plot Junk
    %=======================================================================
    % set(gcf,'Visible','on');
    % set(0,'DefaultFigureVisible','on');
    clf; hold on;

    scatter(x(1),x(2),'red','o');
    for ii = 1:((length(x)-3)/2)
        scatter(x((ii-1)*2 + 4),x((ii-1)*2 + 5),'blue','x');
    end

    
    % Plot "unofficial" landmarks
    idx = find(landmark_list(:,4) == 0);
    scatter(landmark_list(idx,1),landmark_list(idx,2),[],[.5 .5 .5],'x');
    
    if(~isempty(observed_LL))
        % Plot observed landmarks
        scatter(landmark_list(idx2,1),landmark_list(idx2,2),'o','b');
    end
    
    %Plot scan data
    %scatter(landmark_list(:,1),landmark_list(:,2)); 
    cartes_data = readCartesian(laserData); %read cartesian co-ordinates
    rot = [cosd(x(3)) -sind(x(3)) x(1); sind(x(3)) cosd(x(3)) x(2); 0 0 1];
    tmp = rot*[cartes_data,ones(length(cartes_data),1)]';
    scatter(tmp(1,:),tmp(2,:),'magenta','.');
    axis([-5 5 -5 5]);
    
    drawArrow=@(x,y,varargin) quiver (x(1),y(1),x(2)-x(1),y(2)-y(1),0,varargin{:}); hold on   
    xx=x(1)+.5*cosd(x(3));
    yy=(x(2)+.5*sind(x(3))); 
    x1=[x(1) xx];
    y1=[x(2) yy];
    drawArrow(x1,y1,'linewidth',3,'color','r');
    % End Plot Junk
    %----------------------------------------------------------------------    
    
    hold off
end


