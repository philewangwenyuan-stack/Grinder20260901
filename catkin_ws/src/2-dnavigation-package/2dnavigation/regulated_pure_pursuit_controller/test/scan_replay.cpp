// Offline transformed LaserScan replay. Input: min_x max_x min_y max_y front
// side, polygon_count, polygon XY pairs, then scan XY pairs until EOF.
#include <iostream>
#include <map>
#include "regulated_pure_pursuit_controller/scan_classifier.hpp"
using namespace regulated_pure_pursuit_controller;
int main() {
  double x0,x1,y0,y1,front,side; unsigned count;
  if (!(std::cin>>x0>>x1>>y0>>y1>>front>>side>>count) || count>1024) return 2;
  std::vector<ScanXY> polygon;
  for (unsigned i=0;i<count;++i) { ScanXY p; if (!(std::cin>>p.x>>p.y)) return 2; polygon.push_back(p); }
  std::map<ScanZone,int> hits; double x,y;
  while (std::cin>>x>>y) ++hits[classifyScan(x,y,x0,x1,y0,y1,front,side,polygon)];
  std::cout<<"front="<<hits[ScanZone::front]<<" side="<<hits[ScanZone::side]
    <<" body="<<hits[ScanZone::body]<<" self="<<hits[ScanZone::self]<<"\n";
}
