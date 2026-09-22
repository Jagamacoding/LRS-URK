#pragma once

#include <cstddef>
#include <string>
#include <unordered_set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class VoxelMap
{
public:
    VoxelMap(float voxel_size, float safety_radius);

    bool load(const std::string &map_path);

    bool isOccupied(float x, float y, float z) const;
    bool isBlocked(float x, float y, float z) const;
    bool isInside(float x, float y, float z) const;

    // Tieto funkcie bude pouzivat A*
    bool worldToGrid(
        float x, float y, float z,
        int &ix, int &iy, int &iz) const;

    bool gridToWorld(
        int ix, int iy, int iz,
        float &x, float &y, float &z) const;

    bool isBlockedGrid(
        int ix, int iy, int iz) const;

    std::size_t getOccupiedCount() const;
    std::size_t getInflatedCount() const;
    std::size_t getTotalVoxelCount() const;

    int getNx() const;
    int getNy() const;
    int getNz() const;

    float getVoxelSize() const;
    float getSafetyRadius() const;

private:
    std::size_t gridToIndex(
        int ix, int iy, int iz) const;

    void fillBox(
        float x_min, float x_max,
        float y_min, float y_max,
        float z_min, float z_max);

    void inflateObstacles();

    float voxel_size_;
    float safety_radius_;

    pcl::PointXYZ min_point_;
    pcl::PointXYZ max_point_;

    int nx_;
    int ny_;
    int nz_;

    std::unordered_set<std::size_t> occupied_voxels_;
    std::unordered_set<std::size_t> inflated_voxels_;
};