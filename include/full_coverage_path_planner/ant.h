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

            std::vector<std::vector<_Float64>> personal_pheromone_grid;
            std::vector<std::vector<_Float64>> personal_score_grid;
            float current_path_score;
            int visited_counter;
            int multiple_pass_counter;
            int coverage;

            /**
             * 
             * */
            Ant(Point_t start_point, std::vector<std::vector<_Float64>> pheromone_grid, std::vector<std::vector<_Float64>> score_grid);
            /**
             *
             **/
            std::vector<std::vector<_Float64>> producePheromones(std::vector<std::vector<_Float64>> pheromoneGrid, std::vector<std::vector<_Float64>> costGrid);
            /**
             * @brief 
             * 
             */
            std::list<Point_t> getPossibleMovements(std::vector<std::vector<bool>> visited);

            /**
             * @brief 
             * 
             */
            bool resolveDeadlock(std::vector<std::vector<bool>> visited, std::list<Point_t> goals, std::vector<std::vector<bool>> const& grid);
            /**
             * @brief 
             * 
             */
            void scoreNearbyTiles(std::vector<std::vector<bool>> visited, int accessable_tiles_count);
            /**
             * 
             * */
            bool canMove(std::vector<std::vector<bool>> const& grid,
            std::list<Point_t> to);
            /**
             * 
             * */
            void move(
                Point_t new_location,
                bool add_to_path
            );

            std::list<Point_t> getCurrentPath();
            std::list<gridNode_t> getCurrentPathForAstar();
            Point_t getCurrentLocation();
            float getPheromoneEvaporationRate();

        private:
            std::list<Point_t> current_path;
            Point_t current_location;
            float pheromone_rate;
    };
};