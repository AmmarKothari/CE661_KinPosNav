classdef readDataCSV_SP
    % read data in from SensorPlay app (IPhone)
    properties
        fn
        all_data, data_table
        fspec = '%s%f%f%f%f%f%f%f%f%f%f%f%f%f%f%s%s%f%f%f%f%f%f%f%f'; % how the data is structured in file
        IMU_data, IMU_time, IMU_delta_time
        GPS_data, GPS_time, GPS_delta_time
        Mag_data
        phoneOrientation
        pose_0 = [0,0,0,0,0,0] % pose when you start reading data
        loc_0 % first GPS reading?
        heading
        GPS_sampling_rate = 100 %assume that the GPS reads 100 time slower than IMU
        i_imu=1, i_gps=1 % which reading will be given next
        total_imu, total_gps % total readings of each time
    end
    methods
        function obj = readDataCSV_SP(fn)
            obj.fn = fn;
            fid = fopen(fn, 'r');
            C = textscan(fid, '%s', 1);
            obj.heading = strsplit(C{1}{1}, ',');
            % Read file into table
            obj.data_table = obj.readCSV(fn, obj.fspec);
            % % IMU DATA
            % get timestamps
            [obj.IMU_time, obj.IMU_delta_time] = obj.convert2Time(obj.data_table.Timestamp);
            
            obj.Mag_data = [obj.data_table.magX___T_, obj.data_table.magY___T_, obj.data_table.magZ___T_];
            
            obj.IMU_data = [obj.data_table.accelX, obj.data_table.accelY, obj.data_table.accelZ, ...
                            obj.data_table.gyroX_rad_s_, obj.data_table.gyroY_rad_s_, obj.data_table.gyroZ_rad_s_, ...
                            obj.Mag_data];
            obj.total_imu = size(obj.IMU_data,1);
                        
            % % GPS DATA
            GPS_idx = 1:obj.GPS_sampling_rate:length(obj.IMU_time);
            obj.GPS_time = obj.IMU_time(GPS_idx);
            obj.GPS_delta_time = obj.IMU_delta_time(GPS_idx);
            obj.data_table.Lat = obj.fixNaNs(obj.data_table.Lat);
            obj.data_table.Long = obj.fixNaNs(obj.data_table.Long);
            % Convert Latitude and Longitude to UTM coordinates
            % then it is in meteres in that local plane
            [x,y,~] = deg2utm(obj.data_table.Lat(GPS_idx), obj.data_table.Long(GPS_idx));
            obj.GPS_data = [x,y, obj.data_table.Alt_feet_(GPS_idx)];
            obj.total_gps = size(obj.GPS_data,1);
            
            % Phone Orientation
            obj.phoneOrientation = [obj.data_table.Pitch_rads_, obj.data_table.Roll_rads_, obj.data_table.Yaw_rads_];
%             obj.phoneOrientation_calc =
            
        end
        
        function obj = zeroStart(obj)
            obj.GPS_data = obj.GPS_data - obj.GPS_data(1,:);
        end
        
        function obj = restart(obj)
            % clear indices to restart trial
            obj.i_imu = 1; obj.i_gps = 1;
        end
        
        function [obj, IMUreading, IMUt, IMUdt] = querryIMU(obj, t)
            % see if another measurement is available based on time
            IMUreading = nan;
            IMUt = t;
            IMUdt = 0;
            if obj.i_imu <= obj.total_imu && obj.IMU_time(obj.i_imu) < t
                IMUreading = obj.IMU_data(obj.i_imu,:);
                IMUt = obj.IMU_time(obj.i_imu);
                IMUdt = obj.IMU_delta_time(obj.i_imu);
                obj.i_imu = obj.i_imu+1;
            end
        end
        function [obj, GPSreading, GPSt, GPSdt] = querryGPS(obj, t)
            % see if another measurement is available based on time
            GPSreading = nan;
            GPSt = t;
            GPSdt = 0;
            if obj.i_gps <= obj.total_gps && obj.GPS_time(obj.i_gps) < t
                GPSreading = obj.GPS_data(obj.i_gps,:);
                GPSt = obj.GPS_time(obj.i_gps);
                GPSdt = obj.GPS_delta_time(obj.i_gps);
                obj.i_gps = obj.i_gps+1;
            end
        end
        
        function flag = trialEndCheck(obj)
            % check if all the data points have been read
            if obj.i_imu == obj.total_imu && obj.i_gps == obj.total_gps
                flag = true;
            else
                flag = false;
            end
        end
        
        function obj = plotGPSdata(obj)
            scatter3(obj.GPS_data(:,1), obj.GPS_data(:,2), obj.GPS_data(:,3), 'rx');
            hold on;
            scatter3(obj.GPS_data(1,1), obj.GPS_data(1,2), obj.GPS_data(1,3), 'bo');
            scatter3(obj.GPS_data(end,1), obj.GPS_data(end,2), obj.GPS_data(end,3), 'go');
        end
        
        function obj = plotAcceleration(obj)
            subplot(3,1,1)
            plot(obj.IMU_data(:,1), 'rx'); title('Accelx')
            subplot(3,1,2)
            plot(obj.IMU_data(:,2), 'rx'); title('Accely')
            subplot(3,1,3)
            plot(obj.IMU_data(:,3), 'rx'); title('Accelz')
        end
        
        function obj = plotRotationRates(obj)
            subplot(3,1,1)
            plot(obj.IMU_data(:,4), 'rx'); title('rotation rate around x')
            subplot(3,1,2)
            plot(obj.IMU_data(:,5), 'rx'); title('rotation rate around y')
            subplot(3,1,3)
            plot(obj.IMU_data(:,6), 'rx'); title('rotation rate around z')
        end
        
        function obj = plotPhoneRotation(obj)
            % from phone values
            phi_gt = obj.data_table.Pitch_rads_;
            theta_gt = obj.data_table.Roll_rads_;
            psi_gt = obj.data_table.Yaw_rads_;
            subplot(3,1,1)
            plot(phi_gt, 'rx');
            ylim([-pi/2, pi/2])
            title('pitch')
            subplot(3,1,2)
            plot(theta_gt, 'rx');
            ylim([-pi, pi])
            title('roll')
            subplot(3,1,3)
            plot(psi_gt, 'rx');
            ylim([-pi, pi]) 
            title('yaw')
        end
        
        function obj = plotRotation(obj)
            % from phone values
            phi_gt = obj.data_table.Pitch_rads_;
            theta_gt = obj.data_table.Roll_rads_;
            psi_gt = obj.data_table.Yaw_rads_;
            % calulated values
%             theta = cumsum(obj.IMU_data(1:end-1,5) .* obj.IMU_delta_time) + theta_gt(1);
            theta = atan2(-obj.IMU_data(1:end, 1),obj.IMU_data(1:end, 2));
            theta = obj.wrap(theta, -pi, pi);
%             phi = cumsum(obj.IMU_data(1:end-1,4) .* obj.IMU_delta_time) + phi_gt(1);
            phi = atan2(obj.IMU_data(1:end, 2),-obj.IMU_data(1:end, 1).*sin(theta) + obj.IMU_data(1:end, 3).*cos(theta));
%             phi = obj.wrap(phi, -pi, pi);
%             psi = cumsum(obj.IMU_data(1:end-1,6) .* obj.IMU_delta_time) + psi_gt(1);
            Rx = @(x) [1, 0, 0; 0, cos(x), sin(x); 0, -sin(x), cos(x)];
            Ry = @(x) [cos(x), 0, -sin(x); 0, 1, 0; sin(x), 0, cos(x)];
            mag_rot = [];
            for i = 1:length(theta)
                mag_rot = [mag_rot, Rx(-phi(i)) * Ry(-theta(i)) * obj.Mag_data(i,:)'];
            end
            psi = atan2(mag_rot(1,:), mag_rot(2,:));
%             psi = obj.wrap(psi, -pi, pi);
            subplot(3,1,1)
            plot(phi, 'rx'); 
            title('pitch \phi')
            hold on
            plot(phi_gt, 'bo');
            hold off
            legend('calculated', 'phone')
            subplot(3,1,2)
            plot(theta, 'rx');
            title('roll \theta')
            hold on
            plot(theta_gt, 'bo');
            hold off
            subplot(3,1,3)
            plot(psi, 'rx'); 
            title('yaw \psi')
            hold on
            plot(psi_gt, 'bo');
            hold off;
        end
        
        function obj = plotMagnetometer(obj)
            mag_x = obj.data_table.magX___T_;
            mag_y = obj.data_table.magY___T_;
            mag_z = obj.data_table.magZ___T_;
            subplot(3,1,1)
            plot(mag_x, 'rx'); 
            title('MagX')
            subplot(3,1,2)
            plot(mag_y, 'rx');
            title('MagY')
            subplot(3,1,3)
            plot(mag_z, 'rx'); 
            title('MagZ')
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
        
        function D = fixNaNs(D)
            nan_idxs = find(isnan(D)==1);
            for i = 1:length(nan_idxs)
                val = nan;
                repl_idx = nan_idxs(i);
                while isnan(val)
                    if repl_idx + 1 < length(D)
                        repl_idx = repl_idx + 1;
                    else
                        repl_idx = repl_idx - 1;
                    end
                    val = D(repl_idx);
                end
                D(nan_idxs(i)) = D(repl_idx);
            end
        end
        
        function A_wrap = wrap(A, val_min, val_max)
            A_wrap = A;
            r = val_max - val_min;
            while (sum(A_wrap>val_max) + sum(A_wrap<val_min)) > 0
                A_wrap = A_wrap + r*(A_wrap<val_min);
                A_wrap = A_wrap - r*(A_wrap>val_max);
            end
        end
    end
end