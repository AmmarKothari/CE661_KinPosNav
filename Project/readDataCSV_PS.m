classdef readDataCSV_PS
    properties
        all_data
        IMU_data, IMU_time, IMU_delta_time
        GPS_data, GPS_time, GPS_delta_time
        pose_0 = [0,0,0,0,0,0] % pose when you start reading data
        loc_0 % first GPS reading?
        heading
    end
    methods
        function obj = readDataCSV(fn)
            fid = fopen(fn, 'r');
            C = textscan(fid, '%s', 1);
            obj.heading = strsplit(C{1}{1}, ',');
            % start reading at second row
            obj.all_data = csvread(fn, 1, 0);
            % find all rows where there is GPS data, there is no time entry
            % in the first column
            GPS_rows = obj.all_data(:,1) == 0;
            obj.IMU_data = obj.all_data(~GPS_rows, 1:16);
            obj.GPS_data = obj.all_data(GPS_rows, 17:end-1);
            
            % IMU data can be written out of order so need to reorganize by
            % time
            [~,sort_idx] = sort(obj.IMU_data(:,1));
            obj.IMU_data = obj.IMU_data(sort_idx, :);
            
            % extracting time data
            obj.IMU_time = obj.Unix2Time(obj.IMU_data(:,1));
            obj.IMU_delta_time = obj.delta_time(obj.IMU_time); % seconds
            obj.GPS_time = obj.Unix2Time(obj.GPS_data(:,end));
            obj.GPS_delta_time = obj.delta_time(obj.GPS_time); % seconds
        end
    end
    
    methods (Static)
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