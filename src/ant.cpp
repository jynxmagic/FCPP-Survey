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
    Ant::Ant(gridNode_t start_point, std::vector<std::vector<_Float64>> pheromone_grid, std::vector<std::vector<_Float64>> probability_matrix, int velocity)
    {
        current_location = start_point;
        personal_pheromone_grid = pheromone_grid;
        personal_probability_matrix = probability_matrix;
        ant_velocity = velocity;
    }

    _Float64 Ant::producePheromones(_Float64 evaporation_rate, _Float64 pheromone_rate, _Float64 system_constant)
    {
        _Float64 pheromones = evaporation_rate * (personal_pheromone_grid[current_location.pos.y][current_location.pos.x] + (pheromone_rate * system_constant));
        personal_pheromone_grid[current_location.pos.y][current_location.pos.x] = pheromones;
        return pheromones;
    }

    gridNode_t Ant::getBestMovement(std::list<gridNode_t> possible_movements, std::vector<std::vector<_Float64>> global_pheromone_grid)
    {
        _Float64 bestScore = -INFINITY;
        gridNode_t bestPoint;

        for (gridNode_t point : possible_movements)
        {
            _Float64 pheramone_at_pos = global_pheromone_grid[point.pos.y][point.pos.x];

            if (pheramone_at_pos > bestScore)
            {
                bestScore = pheramone_at_pos;
                bestPoint = point;
            }
        }

        return bestPoint;
    }

    std::list<Pos_proba> Ant::generateProbabilityDistro(std::list<gridNode_t> possible_movements, std::vector<std::vector<_Float64>> global_pheromone_grid)
    {
        std::list<Pos_phera> pheromonesList;
        _Float64 pheramoneSum = 0;
        for (gridNode_t point : possible_movements)
        {
            _Float64 pheramone = personal_probability_matrix[point.pos.y][point.pos.x];
            pheramoneSum += pheramone;
            pheromonesList.push_back({point, pheramone});
        }

        std::list<Pos_proba> probabiltyDistro;
        for (Pos_phera pp : pheromonesList)
        {
            _Float64 probability = pp.phera / pheramoneSum; // probability normalization
            probabiltyDistro.push_back({pp.pos, probability});
        }

        return probabiltyDistro;
    }

    void Ant::calculateProbabilityMatrix(std::vector<std::vector<_Float64>> global_pheromone_grid)
    {
        int i = 0;
        for (std::vector<_Float64> y : personal_probability_matrix)
        {
            int j = 0;
            for (_Float64 x : y)
            {
                personal_probability_matrix[i][j] = personal_pheromone_grid[i][j] / global_pheromone_grid[i][j];
                j++;
            }
            i++;
        }
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
                xt = orig_x + 1;
            }
            if (i == 1)
            {
                xt = orig_x - 1;
            }
            if (i == 2)
            {
                yt = orig_y + 1;
            }
            if (i == 3)
            {
                yt = orig_y - 1;
            }

            if (xt >= 0 && xt < nCols && yt >= 0 && yt < nRows) // ant within bounds
            {
                if (visited[yt][xt] == eNodeOpen) // and location is not part of working memory
                {
                    gridNode_t possible_movement = {{xt, yt}, 0, 0}; // it's possible to move to that location
                    possible_movements.push_back(possible_movement);
                }
            }
            i++;
        }

        return possible_movements;
    }

    bool Ant::resolveDeadlock(std::vector<std::vector<bool>> visited, std::list<Point_t> goals, std::vector<std::vector<bool>> const &grid)
    {
        std::list<gridNode_t> pathNodesForAstar = current_path;

        bool resign = a_star_to_open_space(grid, current_location, 1, visited, goals, pathNodesForAstar);

        if (!resign)
        {
            current_path = pathNodesForAstar;
        }

        // update visited grid from a*
        std::list<gridNode_t>::iterator it;
        for (it = current_path.begin(); it != current_path.end(); ++it)
        {
            visited[it->pos.y][it->pos.x] = eNodeVisited;
        }

        current_location = current_path.back();

        return resign;
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
        int cx = current_location.pos.x;
        int cy = current_location.pos.y;

        int nx = new_location.pos.x;
        int ny = new_location.pos.y;

        if (cx + 1 == nx)
        {
            direction = EAST;
        }
        else if (cx - 1 == nx)
        {
            direction = WEST;
        }
        else if (cy + 1 == ny)
        {
            direction = NORTH;
        }
        else
        {
            direction = SOUTH;
        }

        current_location = new_location; // gridNode_t
        if (add_to_path)
        {
            current_path.push_back(current_location);
        }
    }
}