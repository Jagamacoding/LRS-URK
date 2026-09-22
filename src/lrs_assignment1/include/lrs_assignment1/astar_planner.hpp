#pragma once

#include <vector>

#include "lrs_assignment1/voxel_map.hpp"


struct Point3D
{
    float x;
    float y;
    float z;
};


class AStarPlanner
{
public:
    explicit AStarPlanner(const VoxelMap &map);

    std::vector<Point3D> plan(
        const Point3D &start,
        const Point3D &goal);

    // Zjednodusenie raw A* cesty
    std::vector<Point3D> simplifyPath(
        const std::vector<Point3D> &path) const;

    // Kontrola, ci je cela cesta collision-free
    bool isPathCollisionFree(
        const std::vector<Point3D> &path) const;

private:
    const VoxelMap &map_;

    float heuristic(
        int x1, int y1, int z1,
        int x2, int y2, int z2) const;

    int gridToIndex(
        int x, int y, int z) const;

    void indexToGrid(
        int index,
        int &x, int &y, int &z) const;

    bool transitionFree(
        int x, int y, int z,
        int nx, int ny, int nz) const;

    // Kontrola priamej viditelnosti medzi dvoma bodmi
    bool lineOfSight(
        const Point3D &a,
        const Point3D &b) const;
};