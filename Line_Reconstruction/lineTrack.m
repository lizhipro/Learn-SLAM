classdef lineTrack
    properties
        ViewIds;
        StartPoints;
        EndPoints;
    end
    methods
        function this = lineTrack(viewIds, startPoints, endPoints)
            if nargin == 0
                this.ViewIds = zeros(1, 0, 'uint32');
                this.StartPoints = zeros(0, 2);
                this.EndPoints = zeros(0, 2);
            else
                this.ViewIds = uint32(viewIds(:)');
                this.StartPoints = startPoints;
                this.EndPoints = endPoints;
            end
        end
    end
end