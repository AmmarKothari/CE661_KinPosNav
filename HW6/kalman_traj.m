classdef kalman_traj
    properties
        Phi, H, Q, R
        prior = struct('x', 0, 'P', 0);
        post = struct('x', 0, 'P', 0);
        K
    end
    methods
        function obj = kalman_traj(Phi, Q, H, R)
            obj.Phi = Phi;
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
            obj.K = obj.prior.P*obj.H'*inv(obj.H*obj.prior.P*obj.H' + obj.R);
        end
        
        function obj = estimate(obj, meas)
            obj.post.x = obj.prior.x + obj.K*(meas - obj.H*obj.prior.x);
        end
        
        function obj = errorCovariance(obj)
            obj.post.P = obj.prior.P - obj.K * obj.H * obj.prior.P;
        end
        
    end

end
