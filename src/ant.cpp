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
    Ant::Ant(Point_t start_point, std::vector<std::vector<_Float64>> pheromone_grid, std::vector<std::vector<_Float64>> score_grid)
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

    std::list<Point_t> Ant::getPossibleMovements(std::vector<std::vector<bool>> visited)
    {
        int orig_x = current_location.x;
        int orig_y = current_location.y;

        std::list<Point_t> possible_movements;

        for (int i = 0; i < 4; i++)
        {
            int xt = orig_x;
            int yt = orig_y;

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

            if (visited[xt][yt] == eNodeOpen)
            {
                possible_movements.push_back(Point_t{xt, yt});
            }
        }

        return possible_movements;
    }

    bool Ant::resolveDeadlock(std::vector<std::vector<bool>> visited, std::list<Point_t> goals, std::vector<std::vector<bool>> const& grid)
    {
        Point_t originalLocation = current_location;

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

                    if (grid[xt][yt] == eNodeOpen)
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
                    if(visited[goalPoint.x][goalPoint.y] == eNodeVisited)
                    {
                        multiple_pass_counter++;
                    }
                    else
                    {
                        visited[goalPoint.x][goalPoint.y] = eNodeVisited;
                        coverage++;
                    }
                    visited_counter++;

                }
                current_location = current_test.pos;
                return true;
            }
        }
        return false;
    }

    void Ant::scoreNearbyTiles(std::vector<std::vector<bool>> visited, int accessable_tiles_count)
    {
        std::list<Point_t> possible_movements = getPossibleMovements(visited);

        int orig_x = getCurrentLocation().x;
        int orig_y = getCurrentLocation().y;

        for (Point_t point : possible_movements)
        {
            int xt = point.x;
            int yt = point.y;
            move(point, false);
            visited_counter++;
            coverage++;
            visited[xt][yt] = eNodeVisited;
            _Float64 coveragef = static_cast<_Float64>(coverage) / static_cast<_Float64>(accessable_tiles_count);
            personal_score_grid[xt][yt] = score(coveragef, multiple_pass_counter);
            visited[xt][yt] = eNodeOpen;
            coverage--;
            visited_counter--;
            move(Point_t{orig_x, orig_y}, false);
        }
    }

    bool Ant::canMove(std::vector<std::vector<bool>> const &grid,
                      std::list<Point_t> to)
    {
        for (Point_t point : to)
        {
            if (grid[point.x][point.y] == eNodeOpen)
            {
                return true;
            }
        }
        return false;
    }

    void Ant::move(
        Point_t new_location,
        bool add_to_path)
    {
        current_location = new_location; //Point_t
        if (add_to_path)
        {
            current_path.push_back(current_location);
        }
    }

    std::list<gridNode_t> Ant::getCurrentPathForAstar()
    {
        std::list<gridNode_t> nodes;
        for (Point_t point : current_path)
        {
            nodes.push_back({point,
                             0,
                             0});
        }
        return nodes;
    }

    std::list<Point_t> Ant::getCurrentPath()
    {
        return current_path;
    }

    Point_t Ant::getCurrentLocation()
    {
        return current_location;
    }

    float Ant::getPheromoneEvaporationRate()
    {
        return pheromone_rate;
    }
}