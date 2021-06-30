#ifndef COSTMAP_2D_OBJECT_LAYER_H_
#define COSTMAP_2D_OBJECT_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

#include <dynamic_reconfigure/server.h>
#include <costmap_2d/ObjectPluginConfig.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>

#include <tf2_ros/transform_broadcaster.h>

namespace costmap_2d
{

typedef struct detection_record_t {
    geometry_msgs::PoseStamped detection_pose;
    geometry_msgs::TransformStamped pallet_frame_to_base_link;
    geometry_msgs::TransformStamped pallet_frame_to_map;
    geometry_msgs::TransformStamped base_link_to_map;
    geometry_msgs::TransformStamped base_link_to_odom;
    geometry_msgs::TransformStamped odom_to_map;
    geometry_msgs::TransformStamped detection_to_base_link;
} detection_record;

class ObjectLayer : public CostmapLayer
{
public:
  ObjectLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual ~ObjectLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual bool isDiscretized() { return true; }
  virtual void matchSize();

protected:
    std::vector<std::vector<geometry_msgs::Point>> transformed_clear_spaces_;
    std::vector<detection_record> object_list;
    ros::Subscriber detection_sub_;

    const std::vector<std::vector<geometry_msgs::Point>>& getPalletFootprint();
    void onDetection(const geometry_msgs::PoseStamped::ConstPtr& msg);

private:
  dynamic_reconfigure::Server<ObjectPluginConfig> *dsrv_ = NULL;
  void reconfigureCB(ObjectPluginConfig &config, uint32_t level);
  std::vector<std::vector<geometry_msgs::Point>> pallet_footprint;
  tf2_ros::TransformBroadcaster br;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_OBJECT_LAYER_H_
