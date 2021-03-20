% This class is to create a edge

classdef LandmarkEdge < g2o.core.BaseBinaryEdge
    
    properties(Access = protected)

    end
    
    methods(Access = public)
    
        function this = LandmarkEdge()
            this = this@g2o.core.BaseBinaryEdge(2);
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate();
            landmark = this.edgeVertices{2}.estimate();
            dx = landmark(1:2) - x(1:2);
            
            this.errorZ(1) = norm(dx) - this.z(1);
            this.errorZ(2) = g2o.stuff.normalize_theta(atan2(dx(2), dx(1)) - x(3) - this.z(2));
        end
        
        function linearizeOplus(this)
            
            x = this.edgeVertices{1}.estimate();
            landmark = this.edgeVertices{2}.estimate();
            dx = landmark(1:2) - x(1:2);
            r = norm(dx);
            
            this.J{1} = ...
                [-dx(1)/r -dx(2)/r 0;
                dx(2)/r^2 -dx(1)/r^2 -1];
            this.J{2} = - this.J{1}(1:2, 1:2);
        end        
    end
end