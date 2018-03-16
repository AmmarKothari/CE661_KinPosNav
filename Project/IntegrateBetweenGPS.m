classdef IntegrateBetweenGPS
    properties
        x=0, y=0, z=0
        x_d=0, y_d=0, z_d=0
        x_all, y_all, z_all
    end
    
    methods
        function obj = IntegrateBetweenGPS()
            i = 1;
        end
        
        function obj = updateGPS(obj, GPSmeas)
            obj.x = GPSmeas(1);
            obj.y = GPSmeas(2);
            obj.z = GPSmeas(3);
        end
        
        function obj = updateIMU(obj, IMUmeas, phoneOrientation, dt)
            R = obj.rotationMatrix(phoneOrientation(1), phoneOrientation(2), phoneOrientation(3));
            a_IMU = [IMUmeas(1:3), 1] * R';
            g_vec = obj.gravity(R)';
            a_N = (a_IMU - g_vec)*9.8; %remove gravity acceleration and convert to m/s^2
            obj.x = obj.x + dt*obj.x_d + 1/2*a_N(1)*dt^2;
            obj.y = obj.y + dt*obj.y_d + 1/2*a_N(2)*dt^2;
            obj.z = obj.z + dt*obj.z_d + 1/2*a_N(3)*dt^2;
            obj.x_d = obj.x_d + a_N(1)*dt;
            obj.y_d = obj.y_d + a_N(2)*dt;
            obj.z_d = obj.z_d + a_N(3)*dt;
        end
        
        function obj = saveData(obj)
            obj.x_all = [obj.x_all; obj.x];
            obj.y_all = [obj.y_all; obj.y];
            obj.z_all = [obj.z_all; obj.z];
        end
        
        function R = rotationMatrix(obj, phi, theta, psi)
            Rx = obj.RotX(phi);
            Ry = obj.RotY(theta);
            Rz = obj.RotZ(psi);
            R = Rz * Ry * Rx;
        end
    end
    
    methods (Static)
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
        
        function g_vec = gravity(R)
            %calculates gravity in new frame
            % is the gravity direction correct?
            g_vec = R * [0,0, -1, 1]';
        end
            
    end
end