function nodeStruct = getNodeStruct()
    
    % Select Serial Port
    %==================================================================
    nodeStruct.serPortn = 'COM1';
    % End select serial port
    %------------------------------------------------------------------

    % Landmark Parameters
    %==================================================================
    % Xbee MAC addresses
    nodeStruct.lmID =  {'4152F337', ... % Black node
                        '4152F343', ... % Green node
                        '415634FC', ... % Red node
                        '41563539'};    % Yellow node
    % Landmark Locations
    nodeStruct.lmLoc = [0, 0; ...
                        1, 2; ...
                        2, 2; ...
                        2, 1];
    % End landmark parameters
    %------------------------------------------------------------------

    % Create landmark_list matrix
    %==================================================================
    % Landmark list contains x,y world frame locations, a measurement
    % RSSI value, and a landmark index ID, for retro-fitting into 
    % existing EKF SLAM Code.
    for ii = 1:length(nodeStruct.lmID)
        nodeStruct.landmark_list(ii,:) = [nodeStruct.lmLoc(ii,:),0,ii];
    end
    % End create landmark_list matrix
    %------------------------------------------------------------------

    % Open Serial Port
    if nodeStruct.serPortn == 1
        errordlg('Select valid COM port');
    else
        nodeStruct.serConn = serial(nodeStruct.serPortn, 'TimeOut', 1, ...
            'BaudRate', 9600);
        nodeStruct.serConn.BytesAvailableFcnCount = 1;
        nodeStruct.serConn.BytesAvailableFcnMode = 'byte';
%         nodeStruct.serConn.BytesAvailableFcn = {@serial_callback, landmark_list};
        try
            fopen(nodeStruct.serConn);
        catch e
            errordlg(e.message);
        end       
    end