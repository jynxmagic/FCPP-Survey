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
                                        Point_t &init)
    {
        std::list<Point_t> optimalPath;
        _Float64 scoreOptimal = INFINITY;

        //2. initialize pheromone grid
        std::vector<std::vector<_Float64>> og_grid;
        
        int i =0;
        int accessable_tiles_count = 0;
        for(std::vector<bool> x : grid)
        {
            int j = 0;
            std::vector<_Float64> temps;
            for(bool y : grid[i])
            {
                temps.push_back(1.0);
                j++;
                if(y==eNodeOpen)
                {
                    accessable_tiles_count++;
                }
            }
            og_grid.push_back(temps);
            i++;
        }
        std::vector<std::vector<_Float64>> global_pheromone_grid = og_grid;
        std::vector<std::vector<_Float64>> global_score_grid = og_grid;

        ROS_INFO("GRID BUILT");

        //1. initialize ants (n=300)
        std::list<Ant> ants;

        Point_t start = {init.x, init.y};

        for(int i=0; i<10; ++i)
        {
            std::vector<std::vector<_Float64>> personal_pheromone_grid = og_grid; //TODO check if copy ref or new obj
            std::vector<std::vector<_Float64>> personal_scored_grid = og_grid;
            ants.push_back(Ant(start, personal_pheromone_grid, personal_scored_grid));
        }


        ROS_INFO("ALL ANTS BUILT");
        //foreach ant

        std::list<Ant> ants_completed_tour;

        int antCount = 0;

        for(Ant ant : ants)
        {
            auto visited(grid);
            visited[init.x][init.y]= eNodeVisited;

            std::list<Point_t> goals = map_2_goals(visited, eNodeOpen);

            while(!goals.empty())
            {
                int orig_x = ant.getCurrentLocation().x;
                int orig_y = ant.getCurrentLocation().y;

                std::list<Point_t> possible_movements = ant.getPossibleMovements(visited);

                if(possible_movements.size() == 0) //ant has ran into a corner
                {
                    if(!ant.resolveDeadlock(visited, goals, grid))
                    {
                        //A* cannot find a path to an open space. It's maybe solved?
                        break;
                    }
                    else
                    {
                        orig_x = ant.getCurrentLocation().x;
                        orig_y = ant.getCurrentLocation().y;
                        possible_movements = ant.getPossibleMovements(visited);
                    }
                }

                ant.scoreNearbyTiles(visited, accessable_tiles_count);

                float rng_float = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

                std::list<Pos_proba> proba;

                i=0;//
                _Float64 proba_sum = 0;
                for(Point_t point : possible_movements)
                {
                    _Float64 loc_phera_personal = ant.personal_pheromone_grid[point.x][point.y];
                    _Float64 loc_score_personal = ant.personal_score_grid[point.x][point.y];

                    float sum = 1;

                    for(Ant a : ants_completed_tour)
                    {
                        sum += a.personal_pheromone_grid[point.x][point.y] * a.personal_score_grid[point.x][point.y];
                    }
                    
                    _Float64 probat = 0;
                    probat = loc_phera_personal * loc_score_personal / sum;
                    proba_sum+=probat;
                    proba.push_back({point, probat});
                }

                Point_t new_location;
                                
                float s=0;
                for(Pos_proba pp : proba)
                {
                    s+=pp.proba/proba_sum;
                    if(rng_float <= s)
                    {
                        new_location = pp.pos;
                        break;
                    }
                }
                float pheromone;
                float sumphera = 1;
                for(Ant b : ants_completed_tour)
                {
                    bool t_on_p = false;
                    std::list<Point_t> ct = b.getCurrentPath();
                    for(Point_t t : ct)
                    {
                        if(t.x == new_location.x && t.y == new_location.y)
                        {
                            sumphera += 1/b.getCurrentPath().size();
                        }
                    }
                }
                pheromone = ((1-0.6) * (ant.personal_pheromone_grid[new_location.x][new_location.y])) * sumphera;
                ant.personal_pheromone_grid[new_location.x][new_location.y] = pheromone;

                for(Point_t p : ant.getCurrentPath())
                {
                    if(new_location.x == p.x && new_location.y == p.y)
                    {
                        ant.multiple_pass_counter++;
                    }
                    else
                    {
                        ant.coverage++;
                    }
                }
                visited[new_location.x][new_location.y] = eNodeVisited;
                ant.move(new_location, true);
                ant.visited_counter++;
                goals = map_2_goals(visited, eNodeOpen);
            }

            ROS_INFO("ant %i solved", antCount);
            _Float64 ant_score_final = score(ant.coverage, ant.multiple_pass_counter);

            if(ant_score_final < scoreOptimal)
            {
                optimalPath = ant.getCurrentPath();
                scoreOptimal = ant_score_final;
            }
            antCount++;
        }

        ROS_INFO("OPTIMAL ANT PATH");
        ROS_INFO("PATH LENGTH: %li", optimalPath.size());
        ROS_INFO("SCORE: %f", scoreOptimal);

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

    std::list<Point_t> goalPoints = runACO(grid, startPoint);
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