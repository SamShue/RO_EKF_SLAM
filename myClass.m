classdef myClass < handle
	properties
        
        
		landmark;
        %Array of structures that contains pos, rssi index, addr, fresh
        
        
        A=36;
        n=1.6;
        
        serPortn = '/dev/ttyUSB0';
        %set the name here
        
        serConn
        
	end
	methods
		function h = myClass()
		%default constructor, initalizes the position of each landmark,
        %   the index of each landmark, and the mac address of each
        %   landmark. Also initalizes the serial communication
          delete(instrfindall); 
          
          %TODO: List which color each of the mac addresses are
          
          %landmark #1
          data(1).pos=[0,3.048];
          data(1).index=1;
          data(1).rssi=0; 
          data(1).addr='4152F343';
          data(1).fresh=0; 
          data(1).dist=0;

          %landmark #2
          data(2).pos=[0,-3.048];
          data(2).index=2;
          data(2).rssi=0; 
          data(2).addr='41563539';
          data(2).fresh=0; 
          data(2).dist=0;  
            
          h.landmark = data;
          
          
          if h.serPortn == 1
              errordlg('Select valid COM port');
          else
              h.serConn = serial('/dev/ttyUSB0', 'TimeOut', 0.1,'BaudRate', 9600);
              try
                  fopen(h.serConn);
              catch e
                  errordlg(e.message);
              end       
          end      
        end
                
    end
    
    methods (Static)
    function getSerialData(h)
    %method used to poll the buffer for new data    
        if(h.serConn.BytesAvailable)
            %check if new bytes are available
        [data count msg] = fread(h.serConn,1);
        
        if(numel(data))
            if(data(1)==hex2dec('F0'))
            %check to make sure the messege starts properly
                
                len = fread(h.serConn, 1);   % Get length of packet delivered
                
                bufS = fread(h.serConn, len);
   
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

                for ii=1:size(h.landmark,2)
                    for jj=1:size(addrs,2)
                        if(addrs{jj}==h.landmark(ii).addr)
                            h.landmark(ii).rssi=rssi(jj); 
                            h.landmark(ii).fresh=1; 
                            h.landmark(ii).dist=10^((rssi(jj)-h.A)/(10*h.n));
                        end
                    end 
                end
            end
        end
        end 
    end
    
    
    
    function observedll=getLandmarkRSSI(h)
        %search through the structure of landmarks contained within object
        %h for any landmarks that are 'fresh' (have been observed) and
        %output them to observedll
        
        observedll=[];
        for ii=1: size(h.landmark,2)
            if(h.landmark(ii).fresh==1)
                observedll=[observedll;h.landmark(ii).dist,h.landmark(ii).index];
            end
        end
    end
    
    
    function updateLandmarkListRSSI( h,observedll )
        %function that changes the freshness value of any landmarks that
        %have been observed to 0.
        for ii=1:size(observedll,1)
            for jj=1:size(h.landmark,2)
                if(observedll(ii,2)==h.landmark(jj).index)
                    h.landmark(jj).fresh=0;
                end
            end
        end
    end  
    end
    
end