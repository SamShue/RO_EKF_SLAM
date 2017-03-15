function landmarkStruct = getLandmarkStruct()
    
    % Select Serial Port
    %==================================================================
    landmarkStruct.serPortn = 'COM1';
    % End select serial port
    %------------------------------------------------------------------

    % Landmark Parameters
    %==================================================================
    % Xbee MAC addresses
    lmID =  {'4152F337', ... % Black node
             '4152F343', ... % Green node
             '415634FC', ... % Red node
             '41563539'};    % Yellow node
    % Landmark Locations
    lmLoc = [0, 0; ...
             1, 2; ...
             2, 2; ...
             2, 1];
    % End landmark parameters
    %------------------------------------------------------------------

    % Open Serial Port
    if landmarkStruct.serPortn == 1
        errordlg('Select valid COM port');
    else
        landmarkStruct.serConn = serial(landmarkStruct.serPortn, 'TimeOut', 1, ...
            'BaudRate', 9600);
        landmarkStruct.serConn.BytesAvailableFcnCount = 1;
        landmarkStruct.serConn.BytesAvailableFcnMode = 'byte';
%         nodeStruct.serConn.BytesAvailableFcn = {@serial_callback, landmark_list};
        try
            fopen(landmarkStruct.serConn);
        catch e
            errordlg(e.message);
        end       
    end