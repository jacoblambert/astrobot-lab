// Read-only geometry of the exact installed preprocessing/covariance pipeline.
#define GLIM_ROS2
#include <fstream>
#include <filesystem>
#include <Eigen/Eigenvalues>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <glim/util/config.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/ros_cloud_converter.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

int main(int argc,char** argv) {
  if(argc!=4 || std::filesystem::exists(argv[3])) return 2;
  try {
    glim::GlobalConfig::instance(argv[1]);
    glim::Config sensors(glim::GlobalConfig::get_config_path("config_sensors"));
    const auto extrinsic=sensors.param<Eigen::Isometry3d>("sensors","T_lidar_imu",Eigen::Isometry3d::Identity()).inverse();
    glim::CloudPreprocessorParams params;
    if(params.num_threads!=1 || !params.global_shutter) throw std::runtime_error("expected serial rigid-scan fixture");
    glim::CloudPreprocessor preprocess(params);
    glim::CloudCovarianceEstimation covariance(1);
    glim::TimeKeeper time;
    std::ofstream output(argv[3]);
    rosbag2_cpp::Reader reader; reader.open(argv[2]);
    while(reader.has_next()) {
      auto data=reader.read_next();
      if(data->topic_name!="/astrobot_0/lidar_0/pointcloud/filtered") continue;
      rclcpp::SerializedMessage serialized(*data->serialized_data);
      sensor_msgs::msg::PointCloud2 msg;
      rclcpp::Serialization<decltype(msg)>().deserialize_message(&serialized,&msg);
      auto raw=glim::extract_raw_points(msg,"","");
      if(!raw || !time.process(raw)) throw std::runtime_error("invalid cloud");
      auto frame=preprocess.preprocess(raw);
      auto points=frame->points;
      for(auto& p:points) p=extrinsic*p;
      std::vector<Eigen::Vector4d> normals;
      std::vector<Eigen::Matrix4d> covs;
      covariance.estimate(points,frame->neighbors,normals,covs);
      Eigen::Matrix3d scatter=Eigen::Matrix3d::Zero();
      Eigen::Matrix<double,6,6> information=Eigen::Matrix<double,6,6>::Zero();
      for(size_t i=0;i<points.size();++i) {
        const Eigen::Vector3d n=normals[i].head<3>();
        Eigen::Matrix<double,6,1> j; j<<points[i].head<3>().cross(n),n;
        scatter+=n*n.transpose(); information+=j*j.transpose();
      }
      if(points.empty()) throw std::runtime_error("empty geometry");
      scatter/=points.size(); information/=points.size();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> normal_eigen(scatter);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> pose_eigen(information);
      std::vector<double> n(normal_eigen.eigenvalues().data(),normal_eigen.eigenvalues().data()+3);
      std::vector<double> h(pose_eigen.eigenvalues().data(),pose_eigen.eigenvalues().data()+6);
      output<<nlohmann::json({{"stamp_ns",int64_t(std::llround(raw->stamp*1e9))},
        {"points",points.size()},{"normal_eigenvalues",n},{"point_plane_information_eigenvalues",h}}).dump()<<'\n';
    }
    if(!output) throw std::runtime_error("geometry write failed");
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
