//
// Copyright [2022] Manchester Metropolitian University"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/ant.h"

namespace full_coverage_path_planner
{
    Ant::Ant(gridNode_t start_point, std::vector<std::vector<_Float64>> pheromone_grid, std::vector<std::vector<_Float64>> score_grid)
    {
        current_location = start_point;
        personal_pheromone_grid = pheromone_grid;
        personal_score_grid = score_grid;
    }

    std::vector<std::vector<_Float64>> Ant::producePheromones(
        std::vector<std::vector<_Float64>> pheromoneGrid,
        std::vector<std::vector<_Float64>> costGrid)
    {
        return pheromoneGrid;
    }

    std::list<gridNode_t> Ant::getPossibleMovements(std::vector<std::vector<bool>> visited, int loc_x, int loc_y)
    {
        uint dx, dy, dx_prev, nRows = visited.size(), nCols = visited[0].size();

        int orig_x = loc_x;
        int orig_y = loc_y;

        std::list<gridNode_t> possible_movements;

        for (int i = 0; i < 4;)
        {
            int xt = orig_x;
            int yt = orig_y;

            if (i == 0)
            {
                xt = orig_x +1;
            }
            if (i == 1)
            {
                xt = orig_x -1;
            }
            if (i == 2)
            {
                yt = orig_y + 1;
            }
            if (i == 3)
            {
                yt = orig_y - 1;
            }

            if (visited[yt][xt] == eNodeOpen)
            {
                gridNode_t possible_movement = {{xt, yt}, 0, 0};
                possible_movements.push_back(possible_movement);
            }

            i++;
        }

        return possible_movements;
    }

    bool Ant::resolveDeadlock(std::vector<std::vector<bool>> visited, std::list<Point_t> goals, std::vector<std::vector<bool>> const& grid)
    {
        std::list<gridNode_t> pathNodesForAstar = current_path;

        bool resign = a_star_to_open_space(grid, current_location, 1, visited, goals, pathNodesForAstar);


        if(!resign)
        {
            current_path = pathNodesForAstar;
        }

        //update visited grid from a*
        std::list<gridNode_t>::iterator it;
        for (it = current_path.begin(); it != current_path.end(); ++it)
        {
            visited[it->pos.y][it->pos.x] = eNodeVisited;
        }

        current_location = current_path.back();

        return resign;

        /*
        Point_t originalLocation = current_location;
        std::list<Point_t> start_path;
        start_path.push_back(current_location);

        std::list<astar_node> openSet;
        std::list<astar_node> closedSet;

        openSet.push_back({originalLocation, 0, 0, 0, start_path});

        Point_t dummy_goal = {current_location.x +4, current_location.y+4};

        int g = 0;
        if(grid[dummy_goal.y][dummy_goal.x] != eNodeOpen)
        {
            return false;
        }
        while(!openSet.empty())
        {
            int goal_value = INT32_MAX;
            astar_node current_node;
            int i = 0;
            int pos = 0;
            g++;
            for(astar_node node : openSet)
            {
                if(node.he < goal_value)
                {
                    current_node = node;
                    pos = i;
                }
                i++;
            }


            std::list<astar_node>::iterator iter = openSet.begin();
            std::advance(iter, pos);
            openSet.erase(iter);

            if(current_node.pos.x == dummy_goal.x && current_node.pos.y == dummy_goal.y)
            {
                //found destination
                for(Point_t point_on_path_to_destination : current_node.path_to_node)
                {
                    current_path.push_back(point_on_path_to_destination);
                    visited[point_on_path_to_destination.y][point_on_path_to_destination.x] = eNodeVisited;
                }
                printGrid(grid, visited, current_path);
                current_location = current_node.pos;
                return true;
            }

            std::list<astar_node> possible_movements;
            for (int i = 0; i < 4; i++)
            {
                int xt = current_node.pos.x;
                int yt = current_node.pos.y;

                if (i == 0)
                {
                    xt++;
                }
                if (i == 1)
                {
                    xt--;
                }
                if (i == 2)
                {
                    yt++;
                }
                if (i == 3)
                {
                    yt--;
                }

                if (grid[yt][xt] == eNodeOpen)
                {
                    std::list<Point_t> path_to_node = current_node.path_to_node;
                    path_to_node.push_back({xt, yt});
                    int h = distanceSquared({xt, yt}, dummy_goal);
                    astar_node possible_movement = {{xt, yt}, g, h, g+h, path_to_node};
                    bool skip = false;
                    for(astar_node check_heuristic_node : openSet)
                    {
                        if(check_heuristic_node.pos.x == xt && check_heuristic_node.pos.y == yt && check_heuristic_node.he < g+h)
                        {
                            skip = true;
                        }
                    }
                    for(astar_node check_heuristic_node : closedSet)
                    {
                        if(check_heuristic_node.pos.x == xt && check_heuristic_node.pos.y == yt && check_heuristic_node.he < g+h)
                        {
                            skip = true;
                        }
                    }

                    if(skip != true)
                    {
                        openSet.push_back(possible_movement);
                    }
                }
            }

            closedSet.push_back(current_node);
        }

        return false;


 /*       Point_t originalLocation = current_location;

        std::list<astar_node> openSet;
        std::list<Point_t> closedSet;

        std::vector<std::pair<Point_t, int>> sortedGoals;

        for (Point_t goal : goals)
        {
            bool goal_is_already_visited = false;
            for(Point_t point : current_path)
            {
                if(goal.x == point.x && goal.y == point.y)
                {
                    goal_is_already_visited = true;
                }
            }
            if(!goal_is_already_visited)
            {
                int distanceToGoal = distanceSquared(goal, originalLocation);
                sortedGoals.push_back({goal, distanceToGoal});
            }

        }

        sort(sortedGoals.begin(), sortedGoals.end(), [=](std::pair<Point_t, int> &a, std::pair<Point_t, int> &b)
             { return a.second < b.second; });

        for (std::pair<Point_t, int> goalPair : sortedGoals)
        {
            if(goalPair.second == 0)
            {
                continue;
            }
            int g = 0;
            int h = 0;
            std::list<Point_t> path;
            bool found = false;
            openSet.push_back({originalLocation, g, goalPair.second, path});
            astar_node current_test;
            closedSet.clear();
            while (!openSet.empty())
            {
                current_test = openSet.front();
                if (current_test.pos.x == goalPair.first.x && current_test.pos.y == goalPair.first.y)
                {
                    found = true;
                    break; //found path
                }
                closedSet.push_back(current_test.pos);
                openSet.pop_front();
                g++;
                std::list<Point_t> possible_movements;

                for (int i = 0; i < 4; i++)
                {
                    int xt = current_test.pos.x;
                    int yt = current_test.pos.y;

                    if (i == 0)
                    {
                        xt += 1;
                    }
                    if (i == 1)
                    {
                        xt -= 1;
                    }
                    if (i == 2)
                    {
                        yt += 1;
                    }
                    if (i == 3)
                    {
                        yt - 1;
                    }

                    if (grid[yt][xt] == eNodeOpen)
                    {
                        Point_t possible_movement = {xt, yt};
                        bool isInClosedSet = false;
                        for (Point_t closedPoint : closedSet)
                        {
                            if (closedPoint.x == xt && closedPoint.y == yt)
                            {
                                isInClosedSet = true;
                            }
                        }

                        if (!isInClosedSet)
                        {
                            possible_movements.push_back(Point_t{xt, yt});
                        }
                    }
                }

                int smallestDistance = INT16_MAX;
                astar_node locationToMove;

                for (Point_t movement : possible_movements)
                {
                    int distance = distanceSquared(movement, goalPair.first);
                    int gh = distance + g;
                    std::list<Point_t> nodes_path = current_test.path_to_node;
                    nodes_path.push_back(movement);

                    if (gh < smallestDistance)
                    {
                        locationToMove = {movement, g, gh, nodes_path};
                    }

                    openSet.push_back({movement, g, gh, nodes_path});

                }
            }

            if (found)
            {
                for (Point_t goalPoint : current_test.path_to_node)
                {
                    current_path.push_back(goalPoint);
                    if(visited[goalPoint.y][goalPoint.x] == eNodeVisited)
                    {
                        multiple_pass_counter++;
                    }
                    else
                    {
                        visited[goalPoint.y][goalPoint.x] = eNodeVisited;
                        coverage++;
                    }
                    visited_counter++;

                }
                current_location = current_test.pos;
                return true;
            }
        }
        return false; */
    }

    void Ant::scoreNearbyTiles(std::vector<std::vector<bool>> visited, int accessable_tiles_count)
    {

        int orig_x = current_location.pos.x;
        int orig_y = current_location.pos.y;

        std::list<gridNode_t> possible_movements = getPossibleMovements(visited, orig_x, orig_y);

        for (gridNode_t point : possible_movements)
        {
            int xt = point.pos.x;
            int yt = point.pos.y;
            move(point, false);
            visited_counter++;
            coverage++;
            visited[yt][xt] = eNodeVisited;
            _Float64 coveragef = static_cast<_Float64>(coverage) / static_cast<_Float64>(accessable_tiles_count);
            personal_score_grid[yt][xt] = score(coveragef, multiple_pass_counter);
            visited[yt][xt] = eNodeOpen;
            coverage--;
            visited_counter--;
            move({{orig_x, orig_y}, 0, 0}, false);
        }
    }

    bool Ant::canMove(std::vector<std::vector<bool>> const &grid,
                      std::list<gridNode_t> to)
    {
        for (gridNode_t point : to)
        {
            if (grid[point.pos.y][point.pos.x] == eNodeOpen)
            {
                return true;
            }
        }
        return false;
    }

    void Ant::move(
        gridNode_t new_location,
        bool add_to_path)
    {
        current_location = new_location; //gridNode_t
        if (add_to_path)
        {
            current_path.push_back(current_location);
        }
    }
}