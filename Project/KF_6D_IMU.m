classdef KF_6D_IMU
    properties
        Phi, H, Q, R
        prior = struct('x', 0, 'P', 0);
        post = struct('x', 0, 'P', 0);
        K
    end
    methods
        function obj = KF_6D_IMU(Q, H, R)
            obj.H = H;
            obj.Q = Q;
            obj.R = R;
        end
        function obj = setInitialValues(obj, x0, P0)
            obj.post.x = x0;
            obj.post.P = P0;
        end
        
        function [obj, est, cov, K] = update(obj, meas)
            obj = obj.predict();
            obj = obj.KalmanGain();
            obj = obj.estimate(meas);
            obj = obj.errorCovariance();
            est = obj.post.x;
            cov = obj.post.P;
            K = obj.K;
        end
        
        function obj = predict(obj)
            obj.prior.x = obj.Phi * obj.post.x;
            obj.prior.P = obj.Phi * obj.post.P*obj.Phi' + obj.Q;
        end
        
        function obj = KalmanGain(obj)
            obj.K = obj.prior.P*obj.H'*(obj.H*obj.prior.P*obj.H' + obj.R);
        end
        
        function obj = estimate(obj, meas)
            obj.post.x = obj.prior.x + obj.K*(meas - obj.H*obj.prior.x);
        end
        
        function obj = errorCovariance(obj)
            obj.post.P = obj.prior.P - obj.K * obj.H * obj.prior.P;
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
        
        function H = stateMeasurementTransition()
            % returns matrix
            % assume 15 dimension state and 6 Dimension measurement
            H = zeros(6,15);
            c = 1;
            % translation
            for i=1:3:7
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
    end
        

end