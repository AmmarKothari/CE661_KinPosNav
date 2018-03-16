function [DCM_earth_to_local] = ecef2NED_frame(lat,lon)

DCM_earth_to_local = [-sin(lat)*cos(lon) -sin(lat)*sin(lon) cos(lat);...
    -sin(lon) cos(lon) 0;...
    -cos(lat)*cos(lon) -cos(lat)*sin(lon) -sin(lat)];

% pos_local = DCM_earth_to_local * pos;

return