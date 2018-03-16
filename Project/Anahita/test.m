clc
for i=1:length(lat)
    P = deg2rad(lat(i)); L=deg2rad(lon(i));
    DCM = [-sin(P)*cos(L) -sin(P)*sin(L) cos(P);-sin(L) cos(L) 0;-cos(P)*cos(L) -cos(P)*sin(L) -sin(P)];
    r = DCM*[lat(i);lon(i);alt(i)];
    lat_n(i,1) = r(1);
    lon_n(i,1) = r(2);
    alt_n(i,1) = r(3);
end

figure;scatter(lon_n,lat_n,'.r')
figure;scatter(lon,lat,'.b')