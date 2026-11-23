#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"

// Offline candidate policy only. Exact convex-polygon / grid-square overlap,
// including interiors and boundary contact. A soft cost is not a hard hazard;
// every overlapping lethal or unknown cell vetoes the padded footprint.
inline bool footprint_area_clear(const nav2_costmap_2d::Costmap2D& grid,
  double x,double y,double yaw,const std::vector<geometry_msgs::msg::Point>& footprint,
  double margin=0.) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
      !std::isfinite(margin) || margin<0.) return false;
  std::vector<geometry_msgs::msg::Point> world;
  nav2_costmap_2d::transformFootprint(x,y,yaw,footprint,world);
  if (world.size()!=4) return false;  // This probe freezes the rectangular Husky footprint.
  double xmin=std::numeric_limits<double>::infinity(),xmax=-xmin,ymin=xmin,ymax=-xmin;
  for (const auto& p:world) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
    xmin=std::min(xmin,p.x); xmax=std::max(xmax,p.x);
    ymin=std::min(ymin,p.y); ymax=std::max(ymax,p.y);
  }
  const double resolution=grid.getResolution(),ox=grid.getOriginX(),oy=grid.getOriginY();
  const int width=grid.getSizeInCellsX(),height=grid.getSizeInCellsY();
  if (xmin-margin<ox || ymin-margin<oy || xmax+margin>=ox+width*resolution ||
      ymax+margin>=oy+height*resolution) return false;
  const int left=std::max(0,int(std::floor((xmin-margin-ox)/resolution))-1);
  const int right=std::min(width-1,int(std::floor((xmax+margin-ox)/resolution)));
  const int bottom=std::max(0,int(std::floor((ymin-margin-oy)/resolution))-1);
  const int top=std::min(height-1,int(std::floor((ymax+margin-oy)/resolution)));
  for (int cy=bottom;cy<=top;++cy) for (int cx=left;cx<=right;++cx) {
    if (grid.getCost(cx,cy)<nav2_costmap_2d::LETHAL_OBSTACLE) continue;
    const double sx=ox+cx*resolution,sy=oy+cy*resolution;
    // Square axes, followed by the four rectangle edge-normal axes (SAT).
    if (sx-margin>xmax+1e-12 || sx+resolution+margin<xmin-1e-12 ||
        sy-margin>ymax+1e-12 || sy+resolution+margin<ymin-1e-12) continue;
    bool separated=false;
    for (size_t i=0;i<world.size();++i) {
      const auto& a=world[i]; const auto& b=world[(i+1)%world.size()];
      const double nx=-(b.y-a.y),ny=b.x-a.x;
      double lo=std::numeric_limits<double>::infinity(),hi=-lo;
      for (const auto& p:world) {const double v=nx*p.x+ny*p.y;lo=std::min(lo,v);hi=std::max(hi,v);}
      const double center=nx*(sx+.5*resolution)+ny*(sy+.5*resolution);
      // Expanding each hazard square is equivalent to expanding the footprint
      // by a world-axis square. This conservatively contains a radius-margin
      // disk, used below to bound *all* poses between motion samples.
      const double radius=(.5*resolution+margin)*(std::abs(nx)+std::abs(ny));
      if (center+radius<lo-1e-12 || center-radius>hi+1e-12) {separated=true;break;}
    }
    if (!separated) return false;
  }
  return true;
}

// Offline collision veto for one declared linear XY / unwrapped-yaw segment.
// This is not a motion planner, a kinematic check, or safety on unseen terrain.
// For N equal intervals, every body point moves at most
//   (translation_length + circumradius * abs(yaw_change)) / (2*N)
// from its interval midpoint. Check that enclosing Minkowski region, not just
// sampled footprints. Unknown/out-of-map remain hazards. No shortest-yaw wrap:
// the caller must retain the intended winding (including complete rotations).
inline bool footprint_swept_area_clear(const nav2_costmap_2d::Costmap2D& grid,
  double x0,double y0,double yaw0,double x1,double y1,double yaw1,
  const std::vector<geometry_msgs::msg::Point>& footprint) {
  if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(yaw0) ||
      !std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(yaw1) || footprint.size()!=4) return false;
  double radius=0.;
  for (const auto& p:footprint) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
    radius=std::max(radius,std::hypot(p.x,p.y));
  }
  const double travel=std::hypot(x1-x0,y1-y0)+radius*std::abs(yaw1-yaw0);
  // Bound both conservatism (<=1.25cm margin) and total work. Budget exhaustion
  // is a rejection, never a coarser unsafe endpoint-only fallback.
  if (!std::isfinite(travel) || travel>4096*.025) return false;
  const int intervals=std::max(1,int(std::ceil(travel/.025)));
  const double margin=travel/(2*intervals);
  for (int i=0;i<intervals;++i) {
    const double t=(i+.5)/intervals;
    if (!footprint_area_clear(grid,x0+t*(x1-x0),y0+t*(y1-y0),yaw0+t*(yaw1-yaw0),footprint,margin))
      return false;
  }
  return true;
}
