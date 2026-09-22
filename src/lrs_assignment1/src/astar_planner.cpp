#include "lrs_assignment1/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>


struct OpenNode
{
    int x;
    int y;
    int z;

    int index;

    float g;
    float f;
};


struct CompareOpenNode
{
    bool operator()(
        const OpenNode &a,
        const OpenNode &b) const
    {
        return a.f > b.f;
    }
};


AStarPlanner::AStarPlanner(
    const VoxelMap &map)
    : map_(map)
{
}


// ------------------------------------------------------------
// Euklidovska heuristika
// ------------------------------------------------------------

float AStarPlanner::heuristic(
    int x1, int y1, int z1,
    int x2, int y2, int z2) const
{
    float dx = static_cast<float>(x2 - x1);
    float dy = static_cast<float>(y2 - y1);
    float dz = static_cast<float>(z2 - z1);

    return std::sqrt(
               dx * dx +
               dy * dy +
               dz * dz) *
           map_.getVoxelSize();
}


// ------------------------------------------------------------
// 3D index -> 1D index
// ------------------------------------------------------------

int AStarPlanner::gridToIndex(
    int x, int y, int z) const
{
    return x +
           map_.getNx() *
               (y +
                map_.getNy() * z);
}


// ------------------------------------------------------------
// 1D index -> 3D index
// ------------------------------------------------------------

void AStarPlanner::indexToGrid(
    int index,
    int &x, int &y, int &z) const
{
    x = index % map_.getNx();

    int temp =
        index / map_.getNx();

    y = temp % map_.getNy();

    z = temp / map_.getNy();
}


// ------------------------------------------------------------
// Kontrola jedneho kroku A*
// ------------------------------------------------------------

bool AStarPlanner::transitionFree(
    int x, int y, int z,
    int nx, int ny, int nz) const
{
    int dx = nx - x;
    int dy = ny - y;
    int dz = nz - z;

    std::vector<int> xs = {0};
    std::vector<int> ys = {0};
    std::vector<int> zs = {0};

    if (dx != 0)
        xs.push_back(dx);

    if (dy != 0)
        ys.push_back(dy);

    if (dz != 0)
        zs.push_back(dz);

    for (int ox : xs)
    {
        for (int oy : ys)
        {
            for (int oz : zs)
            {
                if (ox == 0 &&
                    oy == 0 &&
                    oz == 0)
                {
                    continue;
                }

                if (map_.isBlockedGrid(
                        x + ox,
                        y + oy,
                        z + oz))
                {
                    return false;
                }
            }
        }
    }

    return true;
}


// ------------------------------------------------------------
// Priama viditelnost medzi dvoma world bodmi
// ------------------------------------------------------------

bool AStarPlanner::lineOfSight(
    const Point3D &a,
    const Point3D &b) const
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dz = b.z - a.z;

    float distance =
        std::sqrt(
            dx * dx +
            dy * dy +
            dz * dz);

    if (distance == 0.0f)
    {
        return !map_.isBlocked(
            a.x, a.y, a.z);
    }

    // Kontrolujeme usecku v krokoch mensich ako voxel.
    // Pri voxeli 0.25 m bude krok 0.0625 m.
    float step =
        map_.getVoxelSize() / 4.0f;

    int samples =
        static_cast<int>(
            std::ceil(distance / step));

    for (int i = 0; i <= samples; ++i)
    {
        float t =
            static_cast<float>(i) /
            static_cast<float>(samples);

        float x =
            a.x + t * dx;

        float y =
            a.y + t * dy;

        float z =
            a.z + t * dz;

        if (map_.isBlocked(x, y, z))
        {
            return false;
        }
    }

    return true;
}


// ------------------------------------------------------------
// Line-of-sight zjednodusenie cesty
// ------------------------------------------------------------

std::vector<Point3D> AStarPlanner::simplifyPath(
    const std::vector<Point3D> &path) const
{
    if (path.size() <= 2)
    {
        return path;
    }

    std::vector<Point3D> simplified;

    std::size_t current = 0;

    simplified.push_back(
        path.front());

    while (current < path.size() - 1)
    {
        // Skusime sa dostat co najdalej priamo
        std::size_t next =
            path.size() - 1;

        while (next > current + 1)
        {
            if (lineOfSight(
                    path[current],
                    path[next]))
            {
                break;
            }

            --next;
        }

        simplified.push_back(
            path[next]);

        current = next;
    }

    return simplified;
}


// ------------------------------------------------------------
// Kontrola celej cesty
// ------------------------------------------------------------

bool AStarPlanner::isPathCollisionFree(
    const std::vector<Point3D> &path) const
{
    if (path.empty())
    {
        return false;
    }

    if (path.size() == 1)
    {
        return !map_.isBlocked(
            path[0].x,
            path[0].y,
            path[0].z);
    }

    for (std::size_t i = 0;
         i + 1 < path.size();
         ++i)
    {
        if (!lineOfSight(
                path[i],
                path[i + 1]))
        {
            return false;
        }
    }

    return true;
}


// ------------------------------------------------------------
// A*
// ------------------------------------------------------------

std::vector<Point3D> AStarPlanner::plan(
    const Point3D &start,
    const Point3D &goal)
{
    std::vector<Point3D> empty_path;

    int sx, sy, sz;
    int gx, gy, gz;

    if (!map_.worldToGrid(
            start.x, start.y, start.z,
            sx, sy, sz))
    {
        std::cerr
            << "START je mimo mapy."
            << std::endl;

        return empty_path;
    }

    if (!map_.worldToGrid(
            goal.x, goal.y, goal.z,
            gx, gy, gz))
    {
        std::cerr
            << "GOAL je mimo mapy."
            << std::endl;

        return empty_path;
    }

    if (map_.isBlockedGrid(sx, sy, sz))
    {
        std::cerr
            << "START je v blokovanom voxeli."
            << std::endl;

        return empty_path;
    }

    if (map_.isBlockedGrid(gx, gy, gz))
    {
        std::cerr
            << "GOAL je v blokovanom voxeli."
            << std::endl;

        return empty_path;
    }

    int total =
        static_cast<int>(
            map_.getTotalVoxelCount());

    const float INF =
        std::numeric_limits<float>::infinity();

    std::vector<float> g_score(
        total,
        INF);

    std::vector<int> parent(
        total,
        -1);

    std::vector<bool> closed(
        total,
        false);

    std::priority_queue<
        OpenNode,
        std::vector<OpenNode>,
        CompareOpenNode>
        open;

    int start_index =
        gridToIndex(sx, sy, sz);

    int goal_index =
        gridToIndex(gx, gy, gz);

    g_score[start_index] = 0.0f;

    open.push(
        {
            sx,
            sy,
            sz,
            start_index,
            0.0f,
            heuristic(
                sx, sy, sz,
                gx, gy, gz)
        });

    int expanded_nodes = 0;

    bool found = false;

    while (!open.empty())
    {
        OpenNode current =
            open.top();

        open.pop();

        if (closed[current.index])
        {
            continue;
        }

        closed[current.index] = true;

        ++expanded_nodes;

        if (current.index == goal_index)
        {
            found = true;
            break;
        }

        // 26 susedov
        for (int dz = -1; dz <= 1; ++dz)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if (dx == 0 &&
                        dy == 0 &&
                        dz == 0)
                    {
                        continue;
                    }

                    int nx =
                        current.x + dx;

                    int ny =
                        current.y + dy;

                    int nz =
                        current.z + dz;

                    if (nx < 0 ||
                        nx >= map_.getNx() ||
                        ny < 0 ||
                        ny >= map_.getNy() ||
                        nz < 0 ||
                        nz >= map_.getNz())
                    {
                        continue;
                    }

                    if (!transitionFree(
                            current.x,
                            current.y,
                            current.z,
                            nx,
                            ny,
                            nz))
                    {
                        continue;
                    }

                    int neighbour_index =
                        gridToIndex(
                            nx,
                            ny,
                            nz);

                    if (closed[neighbour_index])
                    {
                        continue;
                    }

                    float step_cost =
                        std::sqrt(
                            static_cast<float>(
                                dx * dx +
                                dy * dy +
                                dz * dz)) *
                        map_.getVoxelSize();

                    float tentative_g =
                        g_score[current.index] +
                        step_cost;

                    if (tentative_g <
                        g_score[neighbour_index])
                    {
                        g_score[neighbour_index] =
                            tentative_g;

                        parent[neighbour_index] =
                            current.index;

                        float h =
                            heuristic(
                                nx, ny, nz,
                                gx, gy, gz);

                        open.push(
                            {
                                nx,
                                ny,
                                nz,
                                neighbour_index,
                                tentative_g,
                                tentative_g + h
                            });
                    }
                }
            }
        }
    }

    if (!found)
    {
        std::cerr
            << "Cesta neexistuje."
            << std::endl;

        return empty_path;
    }

    std::vector<Point3D> path;

    int current =
        goal_index;

    while (current != -1)
    {
        int x, y, z;

        indexToGrid(
            current,
            x, y, z);

        float wx, wy, wz;

        map_.gridToWorld(
            x, y, z,
            wx, wy, wz);

        path.push_back(
            {wx, wy, wz});

        if (current == start_index)
        {
            break;
        }

        current =
            parent[current];
    }

    std::reverse(
        path.begin(),
        path.end());

    if (!path.empty())
    {
        path.front() = start;
        path.back() = goal;
    }

    std::cout
        << "A* expanded nodes: "
        << expanded_nodes
        << std::endl;

    return path;
}