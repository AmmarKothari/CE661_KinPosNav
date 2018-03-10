classdef IntegrateBetweenGPS
    properties
        
    end
    
    methods
        function obj = IntegrateBetweenGPS()
            i = 1;
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
    end
end