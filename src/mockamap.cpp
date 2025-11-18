#include <chrono>
#include <memory>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mockamap/maps.hpp"

class MockaMapNode : public rclcpp::Node
{
public:
  MockaMapNode() : Node("mockamap")
  {
    seed_ = this->declare_parameter<int>("seed", 4546);
    update_freq_ = this->declare_parameter<double>("update_freq", 1.0);
    resolution_ = this->declare_parameter<double>("resolution", 0.38);
    x_length_ = this->declare_parameter<int>("x_length", 100);
    y_length_ = this->declare_parameter<int>("y_length", 100);
    z_length_ = this->declare_parameter<int>("z_length", 10);

    type_ = this->declare_parameter<int>("type", 1);

    this->declare_parameter<double>("width_min", 0.6);
    this->declare_parameter<double>("width_max", 1.5);
    this->declare_parameter<int>("obstacle_number", 10);
    this->declare_parameter<double>("complexity", 0.142857);
    this->declare_parameter<double>("fill", 0.38);
    this->declare_parameter<int>("fractal", 1);
    this->declare_parameter<double>("attenuation", 0.5);
    this->declare_parameter<double>("road_width", 1.0);
    this->declare_parameter<int>("add_wall_x", 0);
    this->declare_parameter<int>("add_wall_y", 0);
    this->declare_parameter<int>("maze_type", 1);
    this->declare_parameter<int>("numNodes", 10);
    this->declare_parameter<double>("connectivity", 0.5);
    this->declare_parameter<int>("nodeRad", 3);
    this->declare_parameter<int>("roadRad", 2);

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", 1);

    generateMap();

    auto period = std::chrono::duration<double>(1.0 / update_freq_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MockaMapNode::publishMap, this));
  }

private:
  void generateMap()
  {
    double scale = 1.0 / resolution_;
    int size_x = static_cast<int>(x_length_ * scale);
    int size_y = static_cast<int>(y_length_ * scale);
    int size_z = static_cast<int>(z_length_ * scale);

    mocka::Maps::BasicInfo info;
    info.node = this;
    info.sizeX = size_x;
    info.sizeY = size_y;
    info.sizeZ = size_z;
    info.seed = seed_;
    info.scale = scale;
    info.output = &output_;
    info.cloud = &cloud_;

    map_.setInfo(info);
    map_.generate(type_);

    RCLCPP_INFO(this->get_logger(), "Generated mock map type %d", type_);
  }

  void publishMap()
  {
    output_.header.stamp = this->now();
    publisher_->publish(output_);
  }

  mocka::Maps map_;
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  sensor_msgs::msg::PointCloud2 output_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int seed_;
  int x_length_;
  int y_length_;
  int z_length_;
  int type_;
  double resolution_;
  double update_freq_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockaMapNode>());
  rclcpp::shutdown();
  return 0;
}
