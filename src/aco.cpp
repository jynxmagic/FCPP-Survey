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
        //some info on the grid
        uint dx, dy, dx_prev, nRows = grid.size(), nCols = grid[0].size();
        //init optimal score is inf
        _Float64 globalScoreOptimal = INFINITY;

        //2. initialize pheromone grid
        std::vector<std::vector<_Float64>> og_grid(nRows, std::vector<_Float64>(nCols, 1.0));
        
        //count accessable tiles
        int i =0;
        int accessable_tiles_count = 0;
        for(std::vector<bool> y : grid)
        {
            int j = 0;
            for(bool x : grid[i])
            {
                j++;
                if(x==eNodeOpen)
                {
                    accessable_tiles_count++;
                }
            }
            i++;
        }

        std::vector<std::vector<_Float64>> global_pheromone_grid = og_grid;
        std::vector<std::vector<_Float64>> global_score_grid = og_grid;


        ROS_INFO("GRID BUILT");

        //1. initialize ants (n=300)
        std::list<Ant> ants;

        Point_t start = {init.x, init.y};

        for(int i=0; i<100; ++i)
        {
            std::vector<std::vector<_Float64>> personal_pheromone_grid = og_grid; //TODO check if copy ref or new obj
            std::vector<std::vector<_Float64>> personal_scored_grid = og_grid;
            ants.push_back(Ant({start, 0, 0}, personal_pheromone_grid, personal_scored_grid));
        }

        //we will initialize the optimal ant as the first ant in the list to begin.
        
        Ant globalOptimalAnt = ants.front();
        std::list<gridNode_t> globalOptimalPath;

        ROS_INFO("ALL ANTS BUILT");
        //foreach ant

        std::list<Ant> ants_completed_tour;

        for(int optimizationCount = 0; optimizationCount < 100; optimizationCount++)
        {
            ROS_INFO("RUN %i", optimizationCount);
            int optimalAntPos = 0;
            _Float64 scoreOptimal = INFINITY;
            int antCount = 0;

            for(Ant ant : ants)
            {
                auto visited(grid);
                //make sure ant is in init location
                ant.move({{init.x, init.y}, 0, 0}, false);

                std::list<Point_t> goals = map_2_goals(visited, eNodeOpen);

                while(!goals.empty())
                {
                    int orig_x = ant.current_location.pos.x;
                    int orig_y = ant.current_location.pos.y;

                    std::list<gridNode_t> possible_movements = ant.getPossibleMovements(visited);
                    if(possible_movements.size() == 0) //ant has ran into a corner
                    {
                        if(ant.resolveDeadlock(visited, goals, grid))
                        {
                            //A* cannot find a path to an open space. It's maybe solved?
                            break;
                        }
                        else
                        {
                            ant.current_location = ant.current_path.back();
                            orig_x = ant.current_location.pos.x;
                            orig_y = ant.current_location.pos.y;
                            possible_movements = ant.getPossibleMovements(visited);
                        }
                    }

                    gridNode_t new_location;
                    float will_follow_best_path_rng = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

                    _Float64 bestScore = -INFINITY;
                    gridNode_t bestPoint;
                    if(ants_completed_tour.size() != 0)
                    {
                        for(gridNode_t point : possible_movements)
                        {
                            _Float64 sum = 0;
                            for(Ant a : ants_completed_tour)
                            {
                                _Float64 ant_phermone_at_pos = a.personal_pheromone_grid[point.pos.y][point.pos.x];
                                sum += ant_phermone_at_pos;
                            }
                            if(sum > bestScore)
                            {
                                bestScore = sum;
                                bestPoint = point;
                            }
                        }
                    }

                    if(bestScore != -INFINITY 
                    &&  will_follow_best_path_rng > 0.7) //30% of the time, ant follows best path.
                    {
                        new_location = bestPoint;
                    }
                    else
                    {
                        std::list<Pos_phera> pheromonesList;

                        i=0;//
                        _Float64 pheramoneSum = 0;
                        for(gridNode_t point : possible_movements)
                        {
                            _Float64 loc_phera_personal = ant.personal_pheromone_grid[point.pos.y][point.pos.x];

                            float sum = 1;

                            for(Ant a : ants_completed_tour)
                            {
                                sum += a.personal_pheromone_grid[point.pos.y][point.pos.x];
                            }
                            
                            _Float64 pheramone = 0;
                            pheramone = loc_phera_personal / sum;
                            pheramoneSum+=pheramone;
                            pheromonesList.push_back({point, pheramone});
                        }


                        std::list<Pos_proba> probabiltyDistro;
                        for(Pos_phera pp : pheromonesList)
                        {
                            _Float64 probability = pp.phera/pheramoneSum; //probability normalization
                            probabiltyDistro.push_back({pp.pos, probability});
                        }

                        _Float64 probaSum = 0;
                        float proba_rng = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                        for(Pos_proba pp : probabiltyDistro)
                        {
                            probaSum += pp.proba;
                            if(probaSum > proba_rng)
                            {
                                new_location = pp.pos;
                                break;
                            }
                        }
                    }

                    int n = 1;
                    if(ant.current_path.size() > 1)
                    {
                        n = ant.current_path.size();
                    }

                    _Float64 ant_personal_pheromone_at_location = (1-0.6) * (ant.personal_pheromone_grid[new_location.pos.y][new_location.pos.x] * 1/n) + (0.3 * 0.3);

                    ant.personal_pheromone_grid[new_location.pos.y][new_location.pos.x] = ant_personal_pheromone_at_location;
                    visited[new_location.pos.y][new_location.pos.x] = eNodeVisited;
                    ant.move(new_location, true);
                    ant.visited_counter++;
                    goals = map_2_goals(visited, eNodeOpen);
                }

                //recalculate some metrics
                ant.multiple_pass_counter = 0;
                std::list<gridNode_t> closedSet;
                for(gridNode_t nodeOnPath : ant.current_path)
                {
                    for(gridNode_t visitedNode : closedSet)
                    {
                        if(nodeOnPath.pos.x == visitedNode.pos.x && nodeOnPath.pos.y == visitedNode.pos.y)
                        {
                            ant.multiple_pass_counter++;
                        }
                    }
                    closedSet.push_back(nodeOnPath);
                }
                ant.coverage = ant.current_path.size()-ant.multiple_pass_counter;


                ROS_INFO("ant %i solved", antCount);
                _Float64 ant_score_final = goals.size()*1000+ant.multiple_pass_counter;
                ROS_INFO("%f", ant_score_final);

                if(ant_score_final < scoreOptimal)
                {
                    optimalAntPos = antCount;
                    scoreOptimal = ant_score_final;
                    if(ant_score_final < globalScoreOptimal)
                    {
                        globalOptimalAnt = ant;
                        globalOptimalPath = ant.current_path;
                        globalScoreOptimal = ant_score_final;
                    }
                }
                antCount++;
            }

            //add more pheromones to best ants trail.
            std::list<Ant>::iterator antIt = ants.begin();
            std::advance(antIt, optimalAntPos);

            for(gridNode_t it : antIt->current_path)
            {
                _Float64 new_pheromone_value_for_best_ant =
                    2 * antIt->personal_pheromone_grid[it.pos.y][it.pos.x];
                antIt->personal_pheromone_grid[it.pos.y][it.pos.x] = new_pheromone_value_for_best_ant;
            }
        }


        ///end
        ROS_INFO("OPTIMAL ANT PATH");
        ROS_INFO("PATH LENGTH: %li", globalOptimalPath.size());
        ROS_INFO("SCORE: %f", globalScoreOptimal);

        ROS_INFO("building path for planner");

        std::list<Point_t> fullPath;
        std::list<gridNode_t>::iterator it;
        for(it = globalOptimalPath.begin(); it != globalOptimalPath.end(); it++)
        {

            fullPath.push_back(it->pos);
        }

        return fullPath;
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