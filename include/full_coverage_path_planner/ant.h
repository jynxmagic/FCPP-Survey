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
            std::vector<std::vector<_Float64>> personal_probability_matrix;
            float current_path_score;
            int visited_counter;
            int multiple_pass_counter;
            int coverage;
            int ant_velocity;
            Direction direction;
            std::list<gridNode_t> current_path;
            gridNode_t current_location;
            float pheromone_rate;

            /**
             * 
             * */
            Ant(gridNode_t start_point, std::vector<std::vector<_Float64>> pheromone_grid, std::vector<std::vector<_Float64>> score_grid, int velocity);
            /**
             *
             **/
            _Float64 producePheromones(_Float64 evaporation_rate, _Float64 pheromone_rate, _Float64 system_constant);
            /**
             * @brief 
             * 
             */
            void calculateProbabilityMatrix(std::vector<std::vector<_Float64>> global_pheromone_grid);
            /**
             * @brief 
             * 
             */
            std::list<gridNode_t> getPossibleMovements(std::vector<std::vector<bool>> visited, int loc_x, int loc_y);

            /**
             * @brief 
             * 
             */
            bool resolveDeadlock(std::vector<std::vector<bool>> visited, std::list<Point_t> goals, std::vector<std::vector<bool>> const& grid);
            /**
             * 
             * */
            bool canMove(std::vector<std::vector<bool>> const& grid,
            std::list<gridNode_t> to);
            /**
             * 
             * */
            void move(
                gridNode_t new_location,
                bool add_to_path
            );
            /**
             * @brief 
             * 
             */
            gridNode_t getBestMovement(std::list<gridNode_t> possible_movements, std::vector<std::vector<_Float64>> global_pheromone_grid);


            /**
             * @brief 
             * 
             */
            std::list<Pos_proba> generateProbabilityDistro(std::list<gridNode_t> possible_movements, std::vector<std::vector<_Float64>> global_pheromone_grid);
    };
};