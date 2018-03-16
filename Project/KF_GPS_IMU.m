classdef KF_GPS_IMU
    % this class is a kalman filter using the GPS and IMU
    % GPS and IMU update steps are seperate
    % % state: [x,x_d, x_dd, y, y_d, y_dd, z, z_d, z_dd, yaw, yaw_d, pitch,
                % pitch_d, roll, roll_d]
    properties
        Phi, H_GPS, H_IMU, Q, R_GPS, R_IMU
        x_all, y_all, z_all
        prior = struct('x', 0, 'P', 0);
        post = struct('x', 0, 'P', 0);
        K
    end
    methods
        function obj = KF_GPS_IMU(Q, H_GPS, H_IMU, R_GPS, R_IMU)
            obj.H_GPS = H_GPS;
            obj.H_IMU = H_IMU;
            obj.Q = Q;
            obj.R_GPS = R_GPS;
            obj.R_IMU = R_IMU;
        end
        function obj = setInitialValues(obj, x0, P0)
            obj.post.x = x0;
            obj.post.P = P0;
        end
        
        function [obj, est, cov, K] = updateGPS(obj, meas)
            
            obj = obj.predict();
            if nargin==2
                obj = obj.GPSKalmanGain();
                obj = obj.GPSestimate(meas);
                obj = obj.GPSerrorCovariance();
            else
                % no measurement so just move forward in time
                obj.post.x = obj.prior.x;
                obj.post.P = obj.prior.P;
            end
            est = obj.post.x;
            cov = obj.post.P;
            K = obj.K;
        end
        
        function [obj, est, cov, K] = updateIMU(obj, IMUmeas, phoneOrientation)
            R = obj.rotationMatrix(phoneOrientation(1), phoneOrientation(2), phoneOrientation(3));
            % acceleration
            a_IMU = [IMUmeas(1:3)', 1] * R; % rotate into UTM frame
            g_vec = obj.gravity()';
            a_N = (a_IMU - g_vec)*9.8; %remove gravity acceleration and convert to m/s^2
            
            % angular velocity
            w_IMU = [IMUmeas(4:end)', 1] * R;
            
            IMUmeas_rot = [a_N(1:end-1), w_IMU(1:end-1)]';
            obj = obj.predict();
            obj = obj.IMUKalmanGain();
            obj = obj.IMUestimate(IMUmeas_rot);
            obj = obj.IMUerrorCovariance();
            est = obj.post.x;
            cov = obj.post.P;
            K = obj.K;
        end
        
        function obj = predict(obj)
            obj.prior.x = obj.Phi * obj.post.x;
            obj.prior.P = obj.Phi * obj.post.P*obj.Phi' + obj.Q;
        end
        
        function K = KalmanGain(obj, H, R)
            K = obj.prior.P*H'*inv(H*obj.prior.P*H' + R);
        end
        
        function obj = GPSKalmanGain(obj)
            obj.K = obj.KalmanGain(obj.H_GPS, obj.R_GPS);
        end
        
        function obj = IMUKalmanGain(obj)
            obj.K = obj.KalmanGain(obj.H_IMU, obj.R_IMU);
        end
        
        function x = estimate(obj, meas, H)
            x = obj.prior.x + obj.K*(meas - H*obj.prior.x);
        end
        
        function obj = GPSestimate(obj, meas)
            obj.post.x = obj.estimate(meas, obj.H_GPS);
        end
        
        function obj = IMUestimate(obj, meas)
            obj.post.x = obj.estimate(meas, obj.H_IMU);
        end
        
        function P = errorCovariance(obj, H)
            P = obj.prior.P - obj.K * H * obj.prior.P;
        end
        
        function obj = GPSerrorCovariance(obj)
            obj.post.P = obj.errorCovariance(obj.H_GPS);
        end
        
        function obj = IMUerrorCovariance(obj)
            obj.post.P = obj.errorCovariance(obj.H_IMU);
        end
        
        function R = rotationMatrix(obj, phi, theta, psi)
            Rx = obj.RotX(phi);
            Ry = obj.RotY(theta);
            Rz = obj.RotZ(psi);
            R = Ry * Rx * Rz;
        end
        
        function obj = zeroStart(obj)
            obj.x_all = obj.x_all - obj.x_all(1);
            obj.y_all = obj.y_all - obj.y_all(1);
            obj.z_all = obj.z_all - obj.z_all(1);
        end
        function obj = saveData(obj)
            obj.x_all = [obj.x_all; obj.post.x(1)];
            obj.y_all = [obj.y_all; obj.post.x(4)];
            obj.z_all = [obj.z_all; obj.post.x(7)];
        end
    end
    methods (Static)
        
        function Phi = stateTransition(dt)
            % returns matrix, depends on dt between measurements
            % Assume a 15 dimension state vector
            % calculate position with acceleration or only based on velocity?
            Phi = zeros(15,15);
            % translation
            for i=1:3:7
                r = i:i+2;
                Phi(r,r) = [1, dt, dt^2/2;
                            0, 1, dt;
                            0, 0, 1];
            end
            % rotation
            for i=10:2:14
                r = i:i+1;
                Phi(r, r) = [1, dt;
                            0, 1];
            end
        end
        
        function H = GPSstateMeasurementTransition()
            % returns matrix
            % assumes 15 dimension state and 3 Dimension measurement
            H = zeros(3,15);
            % position
            for i = 1:3
                H(i,3*(i-1)+1) = 1;
            end
        end
        
        
        function H = IMUstateMeasurementTransition()
            % returns matrix
            % assume 15 dimension state and 6 Dimension measurement
            H = zeros(6,15);
            c = 1;
            % translation
            for i=1:3:4
                r = i:i+2;
                H(c,r) = [0,0,1];
                c=c+1;
            end
            % rotation
            for i=10:2:14
                r = i:i+1;
                H(c,r) = [0,1];
                c=c+1;
            end
        end
        
        function x0 = initialPos(GPSmeas)
            % returns initial position matrix
            % GPS meas set to x, y, and z position
            % assuming 15 states
            x0 = zeros(15,1);
            % position
            for i = 1:3
                x0(3*(i-1)+1) = GPSmeas(i);
            end
        end
        
        
        function Rx = RotX(gamma)
            s = sin(gamma);
            c = cos(gamma);
            Rx = [
            1,	0,	0,	0;
            0,	c,	-s,	0;
            0,	s,	c,	0;
            0,	0,	0,	1;
            ];
        end

        function Ry = RotY(beta)
            s = sin(beta);
            c = cos(beta);
            Ry = [
            c,		0,	s,	0;
            0,		1,	0,	0;
            -s,	0,	c,	0;
            0,		0,	0,	1
            ];
        end

        function Rz = RotZ(alpha)
            s = sin(alpha);
            c = cos(alpha);
            Rz = [
            c,	-s,	0,	0;
            s,	c,	0,	0;
            0,	0,	1,	0;
            0,	0,	0,	1;
            ];
        end

        function T = Tl(x,y,z)
            T = [
            1,	0,	0,	x;
            0,	1,	0,	y;
            0,	0,	1,	z;
            0,	0,	0,	1;
            ];
        end
        
        function g_vec = gravity()
            g_vec = [0,0, -1, 1]';
        end
        
    end
        

end