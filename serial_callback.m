function serial_callback(serialObject,event,h)

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
       
        for ii=1:size(h.landmark_list,2)
        
            for jj=1:size(addrs,2)
                if(addrs{jj}==h.landmark_list(ii).addr)
                    h.landmark_list(ii).rssi=rssi(jj); 
                    h.landmark_list(ii).fresh=1; 
                    h.landmark_list(ii).dist=10^((rssi(jj)-h.A)/(10*h.n));
                end
            end 
       % disp('---------------------')
       % disp(h.landmark_list(ii).addr);
       % disp(h.landmark_list(ii).dist);
        end
    end
end
end 
