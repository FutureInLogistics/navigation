
#include <algorithm>
#include <costmap_2d/object_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>

#include <costmap_2d/footprint.h>
#include <Eigen/Core>
#include <tf2/utils.h>

#include <ros/ros.h>
#include <tf2/convert.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ObjectLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

ObjectLayer::~ObjectLayer()
{
    if (dsrv_)
        delete dsrv_;
}

void ObjectLayer::onDetection(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    object_list.clear();

    detection_record dr;
    
    try{
        auto now = ros::Time::now();
        geometry_msgs::TransformStamped base_link_to_pallet_frame;
        base_link_to_pallet_frame.transform.rotation = msg->pose.orientation;
        base_link_to_pallet_frame.transform.translation.x = msg->pose.position.x;
        base_link_to_pallet_frame.transform.translation.y = msg->pose.position.y;
        base_link_to_pallet_frame.transform.translation.z = msg->pose.position.z;
        base_link_to_pallet_frame.header.frame_id = "skid_unnamed_base_link";
        base_link_to_pallet_frame.child_frame_id = "skid_unnamed_detection";
        base_link_to_pallet_frame.header.stamp = now; //msg->header.stamp;
        
        dr.detection_pose.pose = msg->pose;
        dr.detection_pose.header.frame_id = msg->header.frame_id; // "skid_unnamed_base_link";
        dr.detection_pose.header.stamp = ros::Time::now();

        dr.base_link_to_map = tf_->lookupTransform("map", "skid_unnamed_base_link", 
            now, ros::Duration(0.2));
        dr.base_link_to_odom = tf_->lookupTransform("skid_unnamed_odom", "skid_unnamed_base_link", 
            now, ros::Duration(0.2));
        dr.odom_to_map = tf_->lookupTransform("map", "skid_unnamed_odom", 
            now, ros::Duration(0.2));
        

        br.sendTransform(base_link_to_pallet_frame);
        
        dr.pallet_frame_to_base_link = tf_->lookupTransform("skid_unnamed_base_link", "skid_unnamed_detection", 
            now, ros::Duration(0.2));

        dr.pallet_frame_to_map = tf_->lookupTransform("map", "skid_unnamed_detection", 
            now, ros::Duration(0.2));

        // dr.base_link_to_map.transform.rotation.x = 0;
        // dr.base_link_to_map.transform.rotation.y = 0;
        // dr.base_link_to_map.transform.rotation.z = 0;
        // dr.base_link_to_map.transform.rotation.w = 1;

        // dr.detection_to_base_link.transform.translation.x = msg->pose.position.x;
        // dr.detection_to_base_link.transform.translation.y = msg->pose.position.y;
        // dr.detection_to_base_link.transform.translation.z = msg->pose.position.z;
        // dr.detection_to_base_link.transform.rotation.w = 1;
        // dr.detection_to_base_link.transform.rotation = msg->pose.orientation;
        // dr.detection_to_base_link.transform.rotation = dr.base_link_to_map.transform.rotation;

        // dr.detection_to_base_link.header.stamp = msg->header.stamp;
        // dr.detection_to_base_link.header.frame_id = ""; 
        // dr.detection_to_base_link.child_frame_id = "skid_unnamed_base_link";

        object_list.push_back(dr);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Cannot locate detection in costmap: %s", ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

}

void ObjectLayer::onInitialize()
{
    ros::NodeHandle g_nh;

    detection_sub_ = g_nh.subscribe("detection", 10, &ObjectLayer::onDetection, this);

    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    dynamic_reconfigure::Server<costmap_2d::ObjectPluginConfig>::CallbackType cb = boost::bind(
        &ObjectLayer::reconfigureCB, this, _1, _2);

    if (dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObjectPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
}

void ObjectLayer::reconfigureCB(costmap_2d::ObjectPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void ObjectLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
             master->getOriginX(), master->getOriginY());
}

const std::vector<std::vector<geometry_msgs::Point>>& ObjectLayer::getPalletFootprint() {
    // Returns a list of convex shapes representing the pallet
    pallet_footprint.clear();

    float pallet_length = 2.5; //1.2
    float left_offset = 0.145f / 2 + 0.2275/2;
    std::vector<Eigen::Vector2f> left_tunnel = {
        Eigen::Vector2f(-pallet_length/2, left_offset + 0.2275/2),
        Eigen::Vector2f(-pallet_length/2, left_offset - 0.2275/2),
        Eigen::Vector2f(pallet_length/2, left_offset - 0.2275/2),
        Eigen::Vector2f(pallet_length/2, left_offset + 0.2275/2),
    };

    float right_offset = -0.145f / 2 - 0.2275/2;
    std::vector<Eigen::Vector2f> right_tunnel = {
        Eigen::Vector2f(-pallet_length/2, right_offset + 0.2275/2),
        Eigen::Vector2f(-pallet_length/2, right_offset - 0.2275/2),
        Eigen::Vector2f(pallet_length/2, right_offset - 0.2275/2),
        Eigen::Vector2f(pallet_length/2, right_offset + 0.2275/2),
    };

    // add left entrance
    pallet_footprint.push_back(std::vector<geometry_msgs::Point>());
    for (size_t i = 0; i < left_tunnel.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = left_tunnel[i](0);
        p.y = left_tunnel[i](1);
        pallet_footprint.back().push_back(p);
    }
    
    // add right entrance
    pallet_footprint.push_back(std::vector<geometry_msgs::Point>());
    for (size_t i = 0; i < right_tunnel.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = right_tunnel[i](0);
        p.y = right_tunnel[i](1);
        pallet_footprint.back().push_back(p);
    }
    
    return pallet_footprint;
}

void ObjectLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
    if (!enabled_) {
        return;
    }

    auto pallet_footprint = ObjectLayer::getPalletFootprint();

    geometry_msgs::TransformStamped map_to_odom;
    geometry_msgs::TransformStamped odom_to_current_base_link;
    geometry_msgs::TransformStamped base_link_to_odom;
    geometry_msgs::TransformStamped map_to_current_base_link;
    try{
        odom_to_current_base_link = tf_->lookupTransform("skid_unnamed_base_link", "skid_unnamed_odom", 
            ros::Time(0), ros::Duration(0.1));
        odom_to_current_base_link.transform.rotation.x = 0;
        odom_to_current_base_link.transform.rotation.y = 0;
        odom_to_current_base_link.transform.rotation.z = 0;
        odom_to_current_base_link.transform.rotation.w = 1;


        base_link_to_odom = tf_->lookupTransform("skid_unnamed_odom", "skid_unnamed_base_link", 
            ros::Time(0), ros::Duration(0.1));
        base_link_to_odom.transform.rotation.x = 0;
        base_link_to_odom.transform.rotation.y = 0;
        base_link_to_odom.transform.rotation.z = 0;
        base_link_to_odom.transform.rotation.w = 1;

        map_to_odom = tf_->lookupTransform("skid_unnamed_odom", "map", 
            ros::Time(0), ros::Duration(0.1));

        map_to_current_base_link = tf_->lookupTransform("skid_unnamed_base_link", "map", 
            ros::Time(0), ros::Duration(0.1));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Cannot locate detection in costmap: %s", ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

       
    geometry_msgs::TransformStamped costmap_offset;
    costmap_offset.transform.translation.x = this->getSizeInMetersX() / 2;
    costmap_offset.transform.translation.y = this->getSizeInMetersY() / 2;
    costmap_offset.transform.rotation.w = 1;

    transformed_clear_spaces_.clear();

    // for every detected object
    for (int j = 0; j < object_list.size(); j++)
    {

        // for every convex shape segment of the detected object
        for (size_t i = 0; i < pallet_footprint.size(); i++)
        {
            std::vector<geometry_msgs::Point> area;
            geometry_msgs::Point p_local;
            // p_local = object_list[j].detection_pose.pose.position;

            for (size_t k = 0; k < pallet_footprint[i].size(); k++)
            {
                p_local.x = pallet_footprint[i][k].x;
                p_local.y = pallet_footprint[i][k].y;
                p_local.z = pallet_footprint[i][k].z;


                // tf2::doTransform(p_local, p_local, object_list[j].pallet_frame);
                tf2::doTransform(p_local, p_local, object_list[j].pallet_frame_to_base_link);
                // tf2::doTransform(p_local, p_local, object_list[j].pallet_frame_to_base_link);
                tf2::doTransform(p_local, p_local, object_list[j].base_link_to_odom);
                tf2::doTransform(p_local, p_local, object_list[j].odom_to_map);

                // tf2::doTransform(p_local, p_local, odom_to_current_base_link);

                tf2::doTransform(p_local, p_local, map_to_odom);
                p_local.x -= base_link_to_odom.transform.translation.x;
                p_local.y -= base_link_to_odom.transform.translation.y;
                p_local.z -= base_link_to_odom.transform.translation.z;
                tf2::doTransform(p_local, p_local, costmap_offset);
                
                // tf2::doTransform(p_local, p_local, base_link_to_odom);
                area.push_back(p_local);
            }

            transformed_clear_spaces_.push_back(area);

            for (unsigned int i = 0; i < area.size(); i++)
            {
                touch(area[i].x, area[i].y, min_x, min_y, max_x, max_y);
            }
        }
    }
}

void ObjectLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    // reset costmap
    for (int j = min_j; j < max_j; j++)
    {
      for (int i = min_i; i < max_i; i++)
      {
        int index = getIndex(i, j);
        costmap_[index] = costmap_2d::NO_INFORMATION;
      }
    }

    // Clear object space
    for (size_t i = 0; i < transformed_clear_spaces_.size(); i++)
    {
        setConvexPolygonCost(transformed_clear_spaces_[i], costmap_2d::FREE_SPACE);
    }
    
    // apply object_layer to master grid
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}


}  // namespace costmap_2d
