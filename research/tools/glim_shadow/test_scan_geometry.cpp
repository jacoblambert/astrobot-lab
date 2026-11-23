#include <iostream>
#include <vector>
#include <glim/odometry/scan_geometry_information.hpp>

int main() {
  try {
    auto require=[](bool ok,const char* why){if(!ok) throw std::runtime_error(why);};
    std::vector<Eigen::Vector4d> p,n;
    for(int x=-1;x<=1;++x) for(int y=-1;y<=1;++y) {p.emplace_back(x,y,0,1);n.emplace_back(0,0,1,0);}
    auto h=glim::scan_geometry_information(p.data(),n.data(),p.size());
    const Eigen::Matrix<double,6,6> sum=100.*h;
    require((glim::registration_mean_information(sum,100)-h).norm()<1e-12,"mean curvature scale");
    require((glim::registration_mean_information(2.*sum,200)-h).norm()<1e-12,"duplicate registration inflated confidence");
    bool bad_curvature=false;
    try {glim::registration_mean_information(-sum,100);}catch(const std::invalid_argument&){bad_curvature=true;}
    require(bad_curvature,"negative curvature accepted");
    bad_curvature=false;
    try {glim::registration_mean_information(sum,0);}catch(const std::invalid_argument&){bad_curvature=true;}
    require(bad_curvature,"empty registration accepted");
    Eigen::Matrix<double,6,1> expected;
    expected<<2./3,2./3,1e-6,1e-6,1e-6,1.;
    require((h-expected.asDiagonal().toDenseMatrix()).norm()<1e-12,"plane nullspaces are not weak");
    auto pp=p,nn=n;
    pp.insert(pp.end(),p.begin(),p.end());nn.insert(nn.end(),n.begin(),n.end());
    require((h-glim::scan_geometry_information(pp.data(),nn.data(),pp.size())).norm()<1e-12,"duplicates inflated confidence");
    for(auto& v:nn) v=-v;
    require((h-glim::scan_geometry_information(pp.data(),nn.data(),pp.size())).norm()<1e-12,"normal sign changed information");
    const Eigen::Matrix3d r=Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
    Eigen::Matrix<double,6,6> adj=Eigen::Matrix<double,6,6>::Zero();adj.block<3,3>(0,0)=r;adj.block<3,3>(3,3)=r;
    require((glim::registration_mean_information(adj*sum*adj.transpose(),100)-adj*h*adj.transpose()).norm()<1e-10,"registration information chart rotation");
    for(size_t i=0;i<p.size();++i) {p[i].head<3>()=r*p[i].head<3>();n[i].head<3>()=r*n[i].head<3>();}
    require((glim::scan_geometry_information(p.data(),n.data(),p.size())-adj*h*adj.transpose()).norm()<1e-10,"world-axis-dependent information");
    bool rejected=false;
    try {glim::scan_geometry_information(nullptr,nullptr,0);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"empty scan accepted");
    n[0].x()=std::numeric_limits<double>::quiet_NaN();rejected=false;
    try {glim::scan_geometry_information(p.data(),n.data(),p.size());}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"nonfinite normal accepted");
    std::cout<<"Scan geometry contracts passed\n";
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
