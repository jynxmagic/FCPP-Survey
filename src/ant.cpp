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
    std::vector<std::vector<float>> Ant::producePheromones(
        std::vector<std::vector<float>> pheromoneGrid,
        std::vector<std::vector<float>> costGrid
    )
    {
        return pheromoneGrid;
    }

    bool Ant::canMove(std::vector<std::vector<bool>> const& grid)
    {
        return true;
    }

    void Ant::move(
        std::vector<std::vector<bool>> const& grid, 
        std::vector<std::vector<float>> pheromoneGrid, 
        std::vector<std::vector<float>> costGrid)
    {
        current_location = {1,2}; //Point_t
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

    float Ant::getCurrentPathScore()
    {
        return current_path_score;
    }
}