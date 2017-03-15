classdef myClass < handle
	properties
        
        %pos, rssi index, addr, fresh
		landmark_list;
        A=24;
        n=1.8;
	end
	methods
		function h = myClass()
		  
            
          data(1).pos=[0,0];
          data(1).index=1;
          data(1).rssi=0; 
          data(1).addr='4152F343';
          data(1).fresh=0; 
          data(1).dist=0;

          data(2).pos=[0,1];
          data(2).index=2;
          data(2).rssi=0; 
          data(2).addr='41563539';
          data(2).fresh=0; 
          data(2).dist=0;  
            
          h.landmark_list = data;
          serPortn = 'COM26';
          
          if serPortn == 1
              errordlg('Select valid COM port');
          else
              serConn = serial(serPortn, 'TimeOut', 1, ...
                  'BaudRate', 9600);
              serConn.BytesAvailableFcnCount = 1;
              serConn.BytesAvailableFcnMode = 'byte';
            % serConn.BytesAvailableFcn = {@serial_callback, landmark_list};
              serConn.BytesAvailableFcn = {@serial_callback,h};
              try
                  fopen(serConn);
              catch e
                  errordlg(e.message);
              end       
          end      
        end      
	end
end