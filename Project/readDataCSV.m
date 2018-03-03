classdef readDataCSV
    properties
        data
    end
    methods
        function obj = readDataCSV(fn)
            fid = fopen(fn, 'r');
            C = textscan(fid, '%s', 1);
            % start reading at second row
            data = csvread(fn, 1, 0);
            
        end
    end
end