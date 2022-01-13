//
// Copyright [2022] Manchester Metropolitian University"  [legal/copyright]
//
#include <list>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <fstream>

using std::string;

#include "full_coverage_path_planner/full_coverage_path_planner.h"
#include "full_coverage_path_planner/ant.h"

namespace full_coverage_path_planner
{
    class AntColonyOptimization: public nav_core::BaseGlobalPlanner, private full_coverage_path_planner::FullCoveragePathPlanner
    {
        public:
            std::list<Point_t> runACO(std::vector<std::vector<bool> > const &grid,
                                        Point_t &init);
        private:
            bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);
            /**
             * @brief  Initialization function for the FullCoveragePathPlanner object
             * @param  name The name of this planner
             * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
             */
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    };
};