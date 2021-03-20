%This class is to create the new landmark in 2D

classdef LandmarkStateVertex < g2o.core.BaseVertex
   
    properties(Access = public)
        landmarkId;
    end
    
    methods(Access = public)
        function this = LandmarkStateVertex(Id)
            this=this@g2o.core.BaseVertex(2);
            this.landmarkId = Id;
        end
        
        function landmarkId = landId(this)
            landmarkId = this.landmarkId;
        end
    end
end