//
// Copyright [2022] Manchester Metropolitian University"  [legal/copyright]
//
#include <list>
#include <string>
#include <vector>
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
#include <ros/ros.h>

using std::string;

#include "full_coverage_path_planner/full_coverage_path_planner.h"

namespace full_coverage_path_planner
{
    class Ant
    {
        public:
            /**
             *
             **/
            std::vector<std::vector<float>> producePheromones(std::vector<std::vector<float>> pheromoneGrid, std::vector<std::vector<float>> costGrid);
            /**
             * 
             * */
            bool canMove(std::vector<std::vector<bool>> const& grid);
            /**
             * 
             * */
            void move(std::vector<std::vector<bool>> const& grid, std::vector<std::vector<float>> pheromoneGrid, std::vector<std::vector<float>> costGrid);

            std::list<Point_t> getCurrentPath();
            Point_t getCurrentLocation();
            float getPheromoneEvaporationRate();
            float getCurrentPathScore();
        private:
            std::list<Point_t> current_path;
            Point_t current_location;
            float pheromone_rate;
            float current_path_score;
    };
};