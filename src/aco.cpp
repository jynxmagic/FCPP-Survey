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
    void AntColonyOptimization::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
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

    std::list<Point_t> AntColonyOptimization::runACO(std::vector<std::vector<bool>> const &grid,
                                                     Point_t &init)
    {
        //some info on the grid
        uint dx, dy, dx_prev, nRows = grid.size(), nCols = grid[0].size();
        //init optimal score is inf
        _Float64 globalScoreOptimal = INFINITY;

        //2. initialize pheromone grid
        std::vector<std::vector<_Float64>> og_grid(nRows, std::vector<_Float64>(nCols, 1.0));

        //count accessable tiles
        int i = 0;
        int accessable_tiles_count = 0;
        for (std::vector<bool> y : grid)
        {
            int j = 0;
            for (bool x : grid[i])
            {
                j++;
                if (x == eNodeOpen)
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

        int velocity = 5;
        for (int i = 0; i < 1000; ++i)
        {
            if(i > 200 && i < 400)
                velocity = 4;
            else if(i >= 400 && i < 500)
                velocity = 3;
            else if(i >= 500 && i < 700)
                velocity = 2;
            else
                velocity = 1;
            
            std::vector<std::vector<_Float64>> personal_pheromone_grid = og_grid;
            std::vector<std::vector<_Float64>> personal_scored_grid = og_grid;
            ants.push_back(Ant({start, 0, 0}, personal_pheromone_grid, personal_scored_grid, velocity));
        }

        //we will initialize the optimal ant as the first ant in the list to begin.

        Ant globalOptimalAnt = ants.front();
        std::list<gridNode_t> globalOptimalPath;

        ROS_INFO("ALL ANTS BUILT");
        //foreach ant

        std::list<Ant> ants_completed_tour;

        int optimalAntPos = 0;
        _Float64 scoreOptimal = INFINITY;
        int antCount = 0;
        for (int optimizationCount = 0; optimizationCount < 10; optimizationCount++)
        {
            ROS_INFO("RUN %i", optimizationCount);
            std::list<gridNode_t> optimalAntPath;
            for (Ant ant : ants)
            {
                auto visited(grid);
                //make sure ant is in init location
                ant.move({{init.x, init.y}, 0, 0}, false);
                //first location is visited
                visited[init.y][init.x] = eNodeVisited;
                
                std::list<Point_t> goals = map_2_goals(visited, eNodeOpen);

                while (!goals.empty())
                {
                    int orig_x = ant.current_location.pos.x;
                    int orig_y = ant.current_location.pos.y;


                    std::list<gridNode_t> possible_movements = ant.getPossibleMovements(visited, orig_x, orig_y);
                    gridNode_t new_location;
                    if(possible_movements.size() == 0) //ant has nowhere to go ! :(
                    {
                        //A* to open path?
                        if (ant.resolveDeadlock(visited, goals, grid))
                        {
                            //A* cannot find a path to an open space. It's maybe solved?
                            break;
                        }
                        else
                        {
                            //A* took the ant to a new path. Go to next loop iteration
                            ant.current_location = ant.current_path.back();
                            new_location = ant.current_path.back();
                        }
                    }
                    else
                    {
                        //ant can make some movements, follow on creating probability distro from pheromones.
                        _Float64 bestScore = -INFINITY;
                        gridNode_t bestPoint;
                        if (antCount != 0)
                        {
                            for (gridNode_t point : possible_movements)
                            {
                                _Float64 pheramone_at_pos = global_pheromone_grid[point.pos.y][point.pos.x];

                                if (pheramone_at_pos > bestScore)
                                {
                                    bestScore = pheramone_at_pos;
                                    bestPoint = point;
                                }
                            }
                        }

                        float will_follow_best_path_rng = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
                        if (bestScore != -INFINITY

                            && will_follow_best_path_rng > 0.3) //70% of the time, ant follows best path.
                        {
                            new_location = bestPoint;
                        }
                        else
                        {
                            std::list<Pos_phera> pheromonesList;
                            i = 0; //
                            _Float64 pheramoneSum = 0;
                            for (gridNode_t point : possible_movements)
                            {
                                _Float64 loc_phera_personal = ant.personal_pheromone_grid[point.pos.y][point.pos.x];
                                _Float64 loc_pehra_global = global_pheromone_grid[point.pos.y][point.pos.x];

                                _Float64 pheramone = 0;
                                pheramone = loc_phera_personal / loc_pehra_global;
                                pheramoneSum += pheramone;
                                pheromonesList.push_back({point, pheramone});
                            }

                            std::list<Pos_proba> probabiltyDistro;
                            for (Pos_phera pp : pheromonesList)
                            {
                                _Float64 probability = pp.phera / pheramoneSum; //probability normalization
                                probabiltyDistro.push_back({pp.pos, probability});
                            }


                            _Float64 probaSum = 0;
                            float proba_rng = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
                            for (Pos_proba pp : probabiltyDistro)
                            {
                                probaSum += pp.proba;
                                if (probaSum > proba_rng)
                                {
                                    new_location = pp.pos;
                                    break;
                                }
                            }
                        }
                    }

                    //move ant
                    visited[new_location.pos.y][new_location.pos.x] = eNodeVisited;
                    ant.move(new_location, true);
                    ant.visited_counter++;
                    goals = map_2_goals(visited, eNodeOpen);

                    //move ant extra according to velocity
                    int moved = 1;
                    int nx = new_location.pos.x;
                    int ny = new_location.pos.y;

                    while (moved != ant.velocity)
                    {
                        if (ant.direction == NORTH)
                        {
                            ny++;
                        }
                        if (ant.direction == EAST)
                        {
                            nx++;
                        }
                        if (ant.direction == SOUTH)
                        {
                            ny--;
                        }
                        if (ant.direction == WEST)
                        {
                            nx--;
                        }

                        gridNode_t nPos = {{nx, ny}, 0, 0};

                        if(visited[ny][nx] == eNodeOpen)
                        {
                            ant.move(nPos, true);
                            ant.visited_counter++;
                            visited[ny][nx] = eNodeVisited;
                            moved++;
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                //move ant to it's initial location
                visited[init.y][init.x] = eNodeOpen;
                goals.push_back({init.x, init.y});
                ant.resolveDeadlock(visited, goals, grid);

                //recalculate some metrics
                ant.multiple_pass_counter = 0;
                std::list<gridNode_t> closedSet;
                int pathSize = ant.current_path.size();
                auto test_grid(grid);
                for (gridNode_t nodeOnPath : ant.current_path)
                {
                    if(test_grid[nodeOnPath.pos.y][nodeOnPath.pos.x] == eNodeVisited)
                    {
                        ant.multiple_pass_counter++;
                    }
                    else
                    {
                        test_grid[nodeOnPath.pos.y][nodeOnPath.pos.x] = eNodeVisited;
                        _Float64 ant_personal_pheromone_at_location = 0.3 * ((ant.personal_pheromone_grid[nodeOnPath.pos.y][nodeOnPath.pos.x] * (1 / pathSize)) + (0.3 * 0.3));
                        ant.personal_pheromone_grid[nodeOnPath.pos.y][nodeOnPath.pos.x] = ant_personal_pheromone_at_location;
                        global_pheromone_grid[nodeOnPath.pos.y][nodeOnPath.pos.x] = 0.7 * (ant_personal_pheromone_at_location + (0.6 * global_pheromone_grid[nodeOnPath.pos.y][nodeOnPath.pos.x]));

                    }
                }

                ROS_INFO("ant %i solved", antCount);
                _Float64 ant_score_final = goals.size() * 1000 + ant.multiple_pass_counter;
                ROS_INFO("score, %f, multipass %i", ant_score_final, ant.multiple_pass_counter);

                if (ant_score_final < scoreOptimal)
                {
                    optimalAntPos = antCount;
                    optimalAntPath = ant.current_path;
                    scoreOptimal = ant_score_final;
                    if (ant_score_final < globalScoreOptimal)
                    {
                        globalOptimalAnt = ant;
                        globalOptimalPath = ant.current_path;
                        globalScoreOptimal = ant_score_final;
                    }
                }
                antCount++;

                //reset the path information for next run
                ant.current_path.clear();
            }

            //add more pheromones to best ants trail.
            std::list<Ant>::iterator antIt = ants.begin();
            std::advance(antIt, optimalAntPos);

            for (gridNode_t it : optimalAntPath)
            {
                _Float64 new_pheromone_value_for_best_ant =
                    50 * global_pheromone_grid[it.pos.y][it.pos.x]; //make it so 50x ants took the pheromones on the best ants personal trail. - this is to simulate many ants taking the shorter route, which doesn't actually happen in the fcpp problem.
                antIt->personal_pheromone_grid[it.pos.y][it.pos.x] = new_pheromone_value_for_best_ant;
                //global trail updating formula
                global_pheromone_grid[it.pos.y][it.pos.x] = new_pheromone_value_for_best_ant;
            }
        }

        ///end
        ROS_INFO("OPTIMAL ANT PATH");
        ROS_INFO("PATH LENGTH: %li", globalOptimalPath.size());
        ROS_INFO("SCORE: %f", globalScoreOptimal);

        ROS_INFO("building path for planner");

        std::list<Point_t> fullPath;
        std::list<gridNode_t>::iterator it;
        for (it = globalOptimalPath.begin(); it != globalOptimalPath.end(); it++)
        {

            fullPath.push_back(it->pos);
        }

        return fullPath;
    }

    bool AntColonyOptimization::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                         std::vector<geometry_msgs::PoseStamped> &plan)
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
        std::vector<std::vector<bool>> grid;
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
        spiral_cpp_metrics_.accessible_counter = spiral_cpp_metrics_.visited_counter - spiral_cpp_metrics_.multiple_pass_counter;
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