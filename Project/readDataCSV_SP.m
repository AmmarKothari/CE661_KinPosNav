classdef readDataCSV_SP
    % read data in from SensorPlay app (IPhone)
    properties
        all_data, data_table
        fspec = '%s%f%f%f%f%f%f%f%f%f%f%f%f%f%f%s%s%f%f%f%f%f%f%f%f'; % how the data is structured in file
        IMU_data, IMU_time, IMU_delta_time
        GPS_data, GPS_time, GPS_delta_time
        pose_0 = [0,0,0,0,0,0] % pose when you start reading data
        loc_0 % first GPS reading?
        heading
    end
    methods
        function obj = readDataCSV_SP(fn)
            fid = fopen(fn, 'r');
            C = textscan(fid, '%s', 1);
            obj.heading = strsplit(C{1}{1}, ',');
            % Read file into table
            obj.data_table = obj.readCSV(fn, obj.fspec);
            % get timestamps
            [obj.IMU_time, obj.IMU_delta_time] = obj.convert2Time(obj.data_table.Timestamp);
            obj.GPS_time = obj.IMU_time;
            obj.GPS_delta_time = obj.IMU_delta_time;
            
            obj.IMU_data = [obj.data_table.accelX, obj.data_table.accelY, obj.data_table.accelZ, ...
                            obj.data_table.gyroX_rad_s_, obj.data_table.gyroY_rad_s_, obj.data_table.gyroZ_rad_s_, ];
            obj.GPS_data = [obj.data_table.Lat, obj.data_table.Long];
            
            % IMU data can be written out of order so need to reorganize by
            % time
%             [~,sort_idx] = sort(obj.IMU_data(:,1));
%             obj.IMU_data = obj.IMU_data(sort_idx, :);
        end
    end
    
    methods (Static)
        function data = readCSV(fn, fspec)
            % reads acc, gyro, rotation-pose, lat/long
            data = readtable(fn, 'Delimiter',',','Format',fspec);
        end
        
        function [t, dt] = convert2Time(t_data)
            converted_times = datevec(datenum(t_data)); % datevecs of time
            t = converted_times(:,end-2:end)*[360,60,1]'; %convert to seonds
            dt = diff(t);
        end
        function matlab_time = Unix2Time(unix_time)
            unix_epoch = datenum(1970,1,1,0,0,0);
            matlab_time = unix_time./86400 + unix_epoch; 
        end
        
        function delta = delta_time(vec_matlab_time)
            diff_time = diff(vec_matlab_time);
            delta = etime(datevec(diff_time), zeros(1,6));
        end
    end
end