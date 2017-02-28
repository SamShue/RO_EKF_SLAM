% function [observed_LL, output_landmark_list]= getLandmarkRSSI(landmark_list)

    % Check for connection to serial port
    if(~isempty(landmark_list))
        % Select Serial Port
        %==================================================================
        serPortn = 'COM1';
        % End select serial port
        %------------------------------------------------------------------

        % Landmark Parameters
        %==================================================================

        % Landmark Locations
        lmLoc = [ 0, 0; ...
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
        for ii = 1:length(lmID)
            landmark_list(ii,:) = [lmLoc(ii,:),0,ii];
        end
        % End create landmark_list matrix
        %------------------------------------------------------------------
        
        % Open Serial Port
        if serPortn == 1
            errordlg('Select valid COM port');
        else
            serList = get(handles.portList,'String');
            serPort = serList{serPortn};
            serConn = serial(serPort, 'TimeOut', 1, ...
                'BaudRate', 9600);
            serConn.BytesAvailableFcnCount = 1;
            serConn.BytesAvailableFcnMode = 'byte';
            serConn.BytesAvailableFcn = {@serial_callback, landmark_list};
            try
                fopen(serConn);
            catch e
                errordlg(e.message);
            end       
        end
    else
        
    end
end

function serial_callback(serialObject, event, landmark_list)
    % Xbee MAC addresses
    lmID = {'4152F337', ... % Black node
            '4152F343', ... % Green node
            '415634FC', ... % Red node
            '41563539'};    % Yellow node
        
    [data count msg] = fread(serialObject,1);

    if(numel(data))
        if(data(1)==hex2dec('F0'))
            %fprintf(data(1));
            %feval(s{1}, event, eventStruct, s{2:end});
            len = fread(serialObject, 1);   % Get length of packet delivered
            bufS = fread(serialObject, len);

            mobileAddr = sprintf('%X',bufS(1:4)); % Transmitting mobile node address
            % Parse bufS into rssi values and anchor addresses
            jj = 1;
            addrs = {};
            rssi = [];
            for ii = 5:5:len
                rssi(jj) = bufS(ii);
                addrs(jj) = {sprintf('%X',bufS((ii+1):(ii + 4)))};
                jj = jj + 1;
            end
        end
    end
    % Update landmark_list structure
    for ii = 1:length(addrs)
        index = find(strcmp(lmID, addrs(ii)));
        landmark_list(index,3) = rssi(jj);
    end
% end