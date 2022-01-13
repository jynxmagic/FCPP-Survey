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
    Ant::Ant(Point_t start_point, std::vector<std::vector<float>> pheromone_grid, std::vector<std::vector<float>> score_grid)
    {
        current_location = start_point;
        personal_pheromone_grid = pheromone_grid;
        personal_score_grid = score_grid;
    }

    std::vector<std::vector<float>> Ant::producePheromones(
        std::vector<std::vector<float>> pheromoneGrid,
        std::vector<std::vector<float>> costGrid
    )
    {
        return pheromoneGrid;
    }

    bool Ant::canMove(std::vector<std::vector<bool>> const& grid,
        std::list<Point_t> to)
    {
        for(Point_t point : to)
        {
            if(grid[point.x][point.y] == eNodeOpen)
            {
                return true;
            }
        }
        return false;
    }

    void Ant::move(
        Point_t new_location,
        bool add_to_path
    )
    {
        current_location = new_location; //Point_t
        if(add_to_path)
        {
            current_path.push_back(current_location);
        }
    }

    std::list<gridNode_t> Ant::getCurrentPathForAstar()
    {
        std::list<gridNode_t> nodes;
        for(Point_t point : current_path)
        {
            nodes.push_back({
                point,
                0,
                0
            });
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