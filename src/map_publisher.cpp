#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/impl/kdtree.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <vector>

typedef pcl::PointXYZ PointT;

using namespace std;

class MapPublisher : public rclcpp::Node {
  public:
    MapPublisher(const std::string &file_name) : Node("map_publisher") {
      this->declare_parameter("add_boundary", 0);
      this->declare_parameter("is_bridge", 0);
      this->declare_parameter("downsample_res", 0.1);
      this->declare_parameter("map_offset_x", 0.0);
      this->declare_parameter("map_offset_y", 0.0);
      this->declare_parameter("map_offset_z", 0.0);

      this->get_parameter("add_boundary", add_boundary);
      this->get_parameter("is_bridge", is_bridge);
      this->get_parameter("downsample_res", downsample_res);
      this->get_parameter("map_offset_x", map_offset_x);
      this->get_parameter("map_offset_y", map_offset_y);
      this->get_parameter("map_offset_z", map_offset_z);

      cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_generator/global_cloud", 10);

      pcl::PointCloud<PointT> cloud_temp, cloud;
      int status = pcl::io::loadPCDFile<PointT>(file_name, cloud_temp);
      cloud = cloud_temp;

      if (status == -1) {
        RCLCPP_ERROR(this->get_logger(), "Can't read file.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "SUCCESS LOAD PCD FILE");

      pcl::VoxelGrid<PointT> voxel_sampler;
      if(is_bridge == 1) {
        for (int i = 0; i < cloud.points.size(); ++i) {
          auto pt = cloud.points[i];
          pcl::PointXYZ pr;
          pr.x = pt.x;
          pr.y = -pt.z;
          pr.z = pt.y;
          cloud.points[i] = pr;
        }
      }

      pcl::PointXYZ global_mapmin;
      pcl::PointXYZ global_mapmax;

      pcl::getMinMax3D(cloud, global_mapmin, global_mapmax);

      RCLCPP_INFO(this->get_logger(), "Map bound: x=%f,%f, y=%f,%f, z=%f,%f", global_mapmin.x, global_mapmax.x, global_mapmin.y, global_mapmax.y, global_mapmin.z, global_mapmax.z);

      if(add_boundary == 1) {
        pcl::PointCloud<PointT> cloud_boundary;
        int add_length = 1;
        int max_x = (int)global_mapmax.x + add_length;
        int min_x = (int)global_mapmin.x - add_length;
        int max_y = (int)global_mapmax.y + add_length;
        int min_y = (int)global_mapmin.y - add_length;
        int max_z = (int)global_mapmax.z + add_length;
        int min_z = (int)global_mapmin.z - add_length;

        int box_x = (max_x - min_x) * 10;
        int box_y = (max_y - min_y) * 10;
        int box_z = (max_z - min_z) * 10;

        cloud_boundary.width = (box_x + 1) * (box_y + 1) * (box_z + 1) - (box_x - 1) * (box_y - 1) * (box_z - 1);
        cloud_boundary.height = 1;
        cloud_boundary.points.resize(cloud_boundary.width * cloud_boundary.height);
        int point_count = 0;

        for (float i = min_x; i <= max_x; i += 0.1) {
          for (float j = min_y; j <= max_y; j += 0.1) {
            cloud.push_back(pcl::PointXYZ(i, j, min_z));
            cloud.push_back(pcl::PointXYZ(i, j, max_z));
          }
        }

        for (float k = min_z; k <= max_z; k += 0.1) {
          for (float i = min_x; i <= max_x; i += 0.1) {
            cloud_boundary.points[point_count].x = i;
            cloud_boundary.points[point_count].y = min_y;
            cloud_boundary.points[point_count].z = k;
            point_count++;
            cloud_boundary.points[point_count].x = i;
            cloud_boundary.points[point_count].y = max_y;
            cloud_boundary.points[point_count].z = k;
            point_count++;
            cloud_boundary.points[point_count].x = i;
            cloud_boundary.points[point_count].y = min_y-0.1;
            cloud_boundary.points[point_count].z = k;
            point_count++;
            cloud_boundary.points[point_count].x = i;
            cloud_boundary.points[point_count].y = max_y+0.1;
            cloud_boundary.points[point_count].z = k;
            point_count++;
          }
          for (float j = min_y; j <= max_y; j += 0.1) {
            cloud_boundary.points[point_count].x = min_x;
            cloud_boundary.points[point_count].y = j;
            cloud_boundary.points[point_count].z = k;
            point_count++;
            cloud_boundary.points[point_count].x = max_x;
            cloud_boundary.points[point_count].y = j;
            cloud_boundary.points[point_count].z = k;
            point_count++;
            cloud_boundary.points[point_count].x = min_x-0.1;
            cloud_boundary.points[point_count].y = j;
            cloud_boundary.points[point_count].z = k;
            point_count++;
            cloud_boundary.points[point_count].x = max_x+0.1;
            cloud_boundary.points[point_count].y = j;
            cloud_boundary.points[point_count].z = k;
            point_count++;
          }
        }

        cloud += cloud_boundary;
        RCLCPP_INFO(this->get_logger(), "Add boundary!!!");
      }

      // Downsample
      voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
      voxel_sampler.setInputCloud(cloud.makeShared());
      voxel_sampler.filter(cloud);

      // Process map
      for (int i = 0; i < cloud.points.size(); ++i) {
        cloud.points[i].x += map_offset_x;
        cloud.points[i].y += map_offset_y;
        cloud.points[i].z += map_offset_z;
      }

      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
      pcl::PolygonMesh triangles;

      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);
      msg.header.frame_id = "world";
      RCLCPP_INFO(this->get_logger(), "Map point size: %d", cloud.points.size());

      int count = 0;
      while (rclcpp::ok()) {
        cloud_pub_->publish(msg);
        rclcpp::spin_some(shared_from_this());
        count++;
        if (count > 100) {
          break;
        }
      }
      RCLCPP_INFO(this->get_logger(), "Map published successfully.");
    }
          
  private:
    string file_name;
    
    int add_boundary = 0;
    int is_bridge = 0;
    double downsample_res;
    double map_offset_x,map_offset_y,map_offset_z;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    
    int minus_twopointcloud(pcl::PointCloud<pcl::PointXYZ>& cloud_input, pcl::PointCloud<pcl::PointXYZ>& cloud_input2, pcl::PointCloud<pcl::PointXYZ>& cloud_output)
    {
      //cloud_input minus cloud_input2
      pcl::search::KdTree<pcl::PointXYZ> minus_kdtree;
      minus_kdtree.setInputCloud(cloud_input2.makeShared());
    
      vector<int> pointIdxRadiusSearch;
      vector<float> pointRadiusSquaredDistance;
      for(int i = 0; i < cloud_input.points.size();i++)
      {
        if (minus_kdtree.radiusSearch(cloud_input.points[i], 0.6, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
          continue;
        }
        cloud_output.points.push_back(cloud_input.points[i]);
      }
      
      RCLCPP_INFO(this->get_logger(), "AFTER MINUS, points count = %d", cloud_output.points.size());
    
      return cloud_output.points.size();
    
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run <pkg> map_publisher <file_name>");
    return -1;
  }
  std::string file_name = argv[1];
  auto node = std::make_shared<MapPublisher>(file_name);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}