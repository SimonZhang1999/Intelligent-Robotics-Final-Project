#include <rrt_nav/graph_searcher.h>

void RRTstarPreparatory::initGridMap(double _resolution, Eigen::Vector2d global_xy_l, Eigen::Vector2d global_xy_u, int max_x_id, int max_y_id){
	gl_xl = global_xy_l(0);
    gl_yl = global_xy_l(1);

    gl_xu = global_xy_u(0);
    gl_yu = global_xy_u(1);
    
    GLX_SIZE  = max_x_id;
    GLY_SIZE  = max_y_id;
    GLXY_SIZE = GLX_SIZE * GLY_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXY_SIZE];
    memset(data, 0, GLXY_SIZE * sizeof(uint8_t));
}

void RRTstarPreparatory::setObs(const double coord_x, const double coord_y){
	if( coord_x < gl_xl  || coord_y < gl_yl || coord_x >= gl_xu || coord_y >= gl_yu)
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);     
    
    data[idx_x + idx_y*GLX_SIZE] = 1;
}

void RRTstarPreparatory::setObs(const int idx_x, const int idx_y){    
    
    data[idx_x + idx_y*GLX_SIZE] = 1;
}

bool RRTstarPreparatory::isObsFree(const double coord_x, const double coord_y){

	Eigen::Vector2d pt;
    Eigen::Vector2i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && 
           (data[idx_x + idx_y*GLX_SIZE] < 1));

}
		
Eigen::Vector2d RRTstarPreparatory::coordRounding(const Eigen::Vector2d & coord){
	return gridIndex2coord(coord2gridIndex(coord));
}

Eigen::Vector2d RRTstarPreparatory::coordRounding(const Eigen::Vector2i & idx){
    return gridIndex2coord(idx);
}

Eigen::Vector2d RRTstarPreparatory::gridIndex2coord(const Eigen::Vector2i & index){
	Eigen::Vector2d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;

    return pt;
}

Eigen::Vector2i RRTstarPreparatory::coord2gridIndex(const Eigen::Vector2d & pt){

	Eigen::Vector2i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1);                          
  
    return idx;

}