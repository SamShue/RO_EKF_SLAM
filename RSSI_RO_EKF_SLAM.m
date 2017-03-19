clc;                                %clear command window
clear all;
close all;                          %close all figures
rosshutdown                         %close current ros incase it is already initalized 

% PC Specific environment variables. Comment out when not needed.
% setenv('ROS_HOSTNAME', 'rahul-ThinkPad-S3-Yoga-14');
% setenv('ROS_IP', '192.168.1.104');

% Robot network variables
ipaddress = 'http://192.168.1.16:11311';         %define ipadress of turtlebot
setenv('ROS_MASTER_URI', ipaddress);
rosinit(ipaddress,'NodeHost','192.168.1.132')                  %initate ros using turtlebot IP

%odom = rossubscriber('/robot_pose_ekf/odom_combined');  %initialize a subscriber node to odomotry data
odom = rossubscriber('/odom');

% EKF Parameter Values
C = 0.2;    % Process Noise Constant
Rc = 1;   % Measurement Noise Constant

lmrk=myClass(); %this is an input to the function and can be either empty or full of stuff
ekf_init = 0;
odomTS = 0;
laserTS = 0;

while(1)
    % Receive ROS Topics
    %======================================================================

    lmrk.getSerialData(lmrk);

  
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
        x=zeros(1,3+size(lmrk.landmark,2)*2);
        for jj=1:size(lmrk.landmark,2)
            x((jj-1)*2 + 4) = lmrk.landmark(jj).pos(1);% + normrnd(0,0.1);
            x((jj-1)*2 + 5) = lmrk.landmark(jj).pos(2);% + normrnd(0,0.1);           
        end
        
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
  
    observed_LL = lmrk.getLandmarkRSSI(lmrk);
   
    % Apply measurement update in EKF if landmarks are observed
    if(~isempty(observed_LL))
       
        [numOfLandmarks] = size(observed_LL,1);
        for ii = 1:numOfLandmarks
            % Measurement vector
            z = observed_LL(ii,1);
            % Measurement noise covariance matrix
            R = observed_LL(ii,1)*Rc;
            % Landmark index
            idx = observed_LL(ii,2);
        
            % Apply EKF measurement update
            [x,P] = RO_EKF_SLAM_Measurement(x,P,z,R,idx);
            lmrk.updateLandmarkListRSSI(lmrk,observed_LL);
        end
    
    end
    % End estimate robot's pose
    %----------------------------------------------------------------------
        
    % Plot Junk
    %=======================================================================
    clf; hold on;
    % Plot robot
  
    
    drawRobot(x(1),x(2),x(3),0.25);
 
    % Plot absolute landmarks
    colors=['r','m','y','c'];
    for ii=1:size(lmrk.landmark,2)
       scatter(lmrk.landmark(ii).pos(1),lmrk.landmark(ii).pos(2),colors(ii)); 
    end
    
    % Plot filtered landmarks
    for ii = 1:((length(x)-3)/2)
        scatter(x((ii-1)*2 + 4),x((ii-1)*2 + 5),colors(ii),'x');
    end
   
    %Plot observed landmarks incorperating distance
     if(~isempty(observed_LL))      
        for(mm=1:size(observed_LL,1))
            
            lineptsx= lmrk.landmark(mm).dist*cosd(atan2d((lmrk.landmark(mm).pos(2)-x(2)),(lmrk.landmark(mm).pos(1)-x(1))));
            lineptsy= lmrk.landmark(mm).dist*sind(atan2d((lmrk.landmark(mm).pos(2)-x(2)),(lmrk.landmark(mm).pos(1)-x(1))));
            plot([x(1) lineptsx+x(1)],[x(2) lineptsy+x(2)],colors(mm));
        end
     end
     
     axis([-5,5,-5,5]);
     % End Plot Junk
     hold off
     
     idx=[];

 end
 
 
