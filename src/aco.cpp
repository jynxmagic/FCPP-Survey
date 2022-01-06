//
// Copyright [2022] Manchester Metropolitian University"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/aco.h"
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::AntColonyOptimization, nav_core::BaseGlobalPlanner)

namespace full_coverage_path_planner
{
    void AntColonyOptimization::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if (!initialized_)
        {
            // Create a publisher to visualize the plan
            ros::NodeHandle private_nh("~/");
            ros::NodeHandle nh, private_named_nh("~/" + name);

            plan_pub_ = private_named_nh.advertise<nav_msgs::Path>("plan", 1);
            // Try to request the cpp-grid from the cpp_grid map_server
            cpp_grid_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");

            // Define  robot radius (radius) parameter
            float robot_radius_default = 0.5f;
            private_named_nh.param<float>("robot_radius", robot_radius_, robot_radius_default);
            // Define  tool radius (radius) parameter
            float tool_radius_default = 0.5f;
            private_named_nh.param<float>("tool_radius", tool_radius_, tool_radius_default);
            initialized_ = true;
        }
    }

    std::list<Point_t> AntColonyOptimization::runACO(std::vector<std::vector<bool> > const &grid,
                                        Point_t &init,
                                        int &multiple_pass_counter,
                                        int &visited_counter)
    {
        std::list<Point_t> optimalPath;
        //1. initialize ants (n=300)

        //-------------
        ///ANTS
        //-------------
        //properties##
        //std::list<Point_t> current_path
        //Point_t current_location
        //float Pheromone evaporation rate (0.1)
        ////float current_path_score
        //std::vector<std::vector<bool>> visitedGrid
        //attributes##
        //std::vector<std::vector<int>> producePheromonesForCurrentLocation(pheromoneGrid, costOfMovement) --> returns updated pheromone grid
        //bool canMove(grid) --> any nearby travesable tiles?
        //void move(grid, pheromone_grid, cost_grid)

        //2. initialize pheromone grid
        std::vector<std::vector<float>> pheromoneGrid;
        
        // declare std::vector<std::vector<float>> pheromone_grid;
        // foreach tile in original_grid
        //      pheromone_grid[x][y] = 1

        //foreach ant

        //3. while goals remain
        //goals = map_2_goals(ant.visited, eNodeOpen)

        //3.1 score grid when travesable tiles have been travelled to
        // poss movements
        //for i = 4
        //  x+1, y+1, x-1, y-1
        //  ant.move(x,y) -> moveAnt
        //  score(ant.visited, grid, etc.) ->Score location
        //   poss_movements[i] = score -> store score
        //  ant.move(origX, origY) -> go back

        //3.2 if no traversable tiles, astar to open space
        //if(!ant.canMove)
        // astar_to_open_space

        //3.3 else, move ant to tile with highest probability
        // probability of movement =
        //  for i in poss_movements
        //      pheromone * cost / sum(pheromone * cost)
        // rng_value = rng.getnumberfloatbetween01
        // if rng >=loc 1 & < loc2. go loc 1. elif rng > loc 2 & rng<=lo3, go loc3. etc.

        //3.4 update pheromone at current location
        // pheromone grid = ant.producePheromone(pheromone_grid, poss_movements)
        //endwhile

        //return ant with lowest cost path

        return optimalPath;
    }


    bool AntColonyOptimization::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                            std::vector<geometry_msgs::PoseStamped>& plan)
    {
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }
    else
    {
        ROS_INFO("Initialized!");
    }

    clock_t begin = clock();
    Point_t startPoint;

    /********************** Get grid from server **********************/
    std::vector<std::vector<bool> > grid;
    nav_msgs::GetMap grid_req_srv;
    ROS_INFO("Requesting grid!!");
    if (!cpp_grid_client_.call(grid_req_srv))
    {
        ROS_ERROR("Could not retrieve grid from map_server");
        return false;
    }

    if (!parseGrid(grid_req_srv.response.map, grid, robot_radius_ * 2, tool_radius_ * 2, start, startPoint))
    {
        ROS_ERROR("Could not parse retrieved grid");
        return false;
    }

    #ifdef DEBUG_PLOT
    ROS_INFO("Start grid is:");
    std::list<Point_t> printPath;
    printPath.push_back(startPoint);
    printGrid(grid, grid, printPath);
    #endif

    std::list<Point_t> goalPoints = runACO(grid,
                                                startPoint,
                                                spiral_cpp_metrics_.multiple_pass_counter,
                                                spiral_cpp_metrics_.visited_counter);
    ROS_INFO("naive cpp completed!");
    ROS_INFO("Converting path to plan");

    parsePointlist2Plan(start, goalPoints, plan);
    // Print some metrics:
    spiral_cpp_metrics_.accessible_counter = spiral_cpp_metrics_.visited_counter
                                                - spiral_cpp_metrics_.multiple_pass_counter;
    spiral_cpp_metrics_.total_area_covered = (4.0 * tool_radius_ * tool_radius_) * spiral_cpp_metrics_.accessible_counter;
    ROS_INFO("Total visited: %d", spiral_cpp_metrics_.visited_counter);
    ROS_INFO("Total re-visited: %d", spiral_cpp_metrics_.multiple_pass_counter);
    ROS_INFO("Total accessible cells: %d", spiral_cpp_metrics_.accessible_counter);
    ROS_INFO("Total accessible area: %f", spiral_cpp_metrics_.total_area_covered);

    // TODO(CesarLopez): Check if global path should be calculated repetitively or just kept
    // (also controlled by planner_frequency parameter in move_base namespace)

    ROS_INFO("Publishing plan!");
    publishPlan(plan);
    ROS_INFO("Plan published!");
    ROS_DEBUG("Plan published");

    clock_t end = clock();
    double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
    std::cout << "elapsed time: " << elapsed_secs << "\n";

    return true;
    }
}