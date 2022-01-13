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
        ROS_INFO("RUNACO");
        std::list<Point_t> optimalPath;
        float scoreOptimal;

        //2. initialize pheromone grid
        std::vector<std::vector<float>> og_grid;
        
        int i =0;
        int accessable_tiles = 0;
        for(std::vector<bool> x : grid)
        {
            int j = 0;
            std::vector<float> temps;
            for(bool y : grid[i])
            {
                temps.push_back(1.0);
                j++;
                if(y==eNodeOpen)
                {
                    accessable_tiles++;
                }
            }
            og_grid.push_back(temps);
            i++;
        }
        std::vector<std::vector<float>> global_pheromone_grid = og_grid;
        std::vector<std::vector<float>> global_score_grid = og_grid;

        ROS_INFO("GRID BUILT");

        //1. initialize ants (n=300)
        std::list<Ant> ants;

        Point_t start = {init.x, init.y};

        for(int i=0; i<300; ++i)
        {
            std::vector<std::vector<float>> personal_pheromone_grid = og_grid; //TODO check if copy ref or new obj
            std::vector<std::vector<float>> personal_scored_grid = og_grid;
            ants.push_back(Ant(start, personal_pheromone_grid, personal_scored_grid));
        }


        ROS_INFO("ALL ANTS BUILT");
        //foreach ant

        std::list<Ant> ants_completed_tour;

        int antCount = 0;

        for(Ant ant : ants)
        {
            std::vector<std::vector<bool>> visited = grid;
            visited[init.x][init.y]= eNodeVisited;

            std::list<Point_t> goals = map_2_goals(visited, eNodeOpen);

            int visited_counter = 0;
            int multiple_pass_counter = 0;
            int coverage = 0;

            while(!goals.empty())
            {
                ROS_INFO("goals %li", goals.size());
                int orig_x = ant.getCurrentLocation().x;
                int orig_y = ant.getCurrentLocation().y;
                std::list<Point_t> possible_movements;
                for(int i = 0; i < 4; i++)
                {
                    int xt = orig_x;
                    int yt = orig_y;

                    if(i==0)
                    {
                        xt+=1;
                    }
                    if(i==1)
                    {
                        xt-=1;
                    }
                    if(i==2)
                    {
                        yt+=1;
                    }
                    if(i==3)
                    {
                        yt-1;
                    }

                    if(visited[xt][yt] == eNodeOpen)
                    {
                        possible_movements.push_back(Point_t {xt, yt});
                    }
                }
                for (Point_t point : possible_movements)
                {
                    int xt = point.x;
                    int yt = point.y;
                    ant.move(point, false);
                    visited_counter++;
                    visited[xt][yt] = eNodeVisited;
                    float coveragef = static_cast<float>(coverage)/ static_cast<float>(accessable_tiles);
                    ant.personal_score_grid[xt][yt] = score(coveragef, multiple_pass_counter);
                    visited[xt][yt] = eNodeOpen;
                    coverage--;
                    visited_counter--;
                    ant.move(Point_t {orig_x, orig_y}, false);
                }
                float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

                std::list<Pos_proba> proba;

                i=0;//
                float proba_sum = 0;
                for(Point_t point : possible_movements)
                {
                    float loc_phera_personal = ant.personal_pheromone_grid[point.x][point.y];
                    float loc_score_personal = ant.personal_score_grid[point.x][point.y];

                    float sum = 1;

                    for(Ant a : ants_completed_tour)
                    {
                        sum += a.personal_pheromone_grid[point.x][point.y] * a.personal_score_grid[point.x][point.y];
                    }
                    
                    float probat = 0;
                    probat = loc_phera_personal * loc_score_personal / sum;
                    proba_sum+=probat;
                    proba.push_back({point, probat});
                }

                Point_t new_location;
                                
                float s=0;
                for(Pos_proba pp : proba)
                {
                    s+=pp.proba/proba_sum;
                    if(r <= s)
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
                        multiple_pass_counter++;
                    }
                    else
                    {
                        coverage++;
                    }
                }
                visited[new_location.x][new_location.y] = eNodeVisited;
                ant.move(new_location, true);
                ROS_INFO("Ant %i moving to x:%i, y:%i", antCount, new_location.x, new_location.y);
                visited_counter++;
                goals = map_2_goals(visited, eNodeOpen);
            }

            float ant_score_final = score(coverage, multiple_pass_counter);

            if(ant_score_final < scoreOptimal)
            {
                optimalPath = ant.getCurrentPath();
            }
            antCount++;
        }

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