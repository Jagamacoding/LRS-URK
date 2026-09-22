#include "lrs_assignment1/voxel_map.hpp"

#include <cmath>
#include <iostream>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>


VoxelMap::VoxelMap(float voxel_size, float safety_radius)
    : voxel_size_(voxel_size),
      safety_radius_(safety_radius),
      nx_(0),
      ny_(0),
      nz_(0)
{
}


bool VoxelMap::load(const std::string &map_path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    // Nacitanie PCD mapy
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *cloud) == -1)
    {
        std::cerr << "Nepodarilo sa nacitat mapu: "
                  << map_path << std::endl;
        return false;
    }

    // Bounding box celeho point cloudu
    pcl::getMinMax3D(*cloud, min_point_, max_point_);

    // Rozmery voxeloveho gridu
    nx_ = static_cast<int>(
              std::floor(
                  (max_point_.x - min_point_.x) / voxel_size_)) +
          1;

    ny_ = static_cast<int>(
              std::floor(
                  (max_point_.y - min_point_.y) / voxel_size_)) +
          1;

    nz_ = static_cast<int>(
              std::floor(
                  (max_point_.z - min_point_.z) / voxel_size_)) +
          1;

    occupied_voxels_.clear();
    inflated_voxels_.clear();

    // ---------------------------------------------------------
    // 1. RAW POINT CLOUD -> OBSADENE VOXELY
    // ---------------------------------------------------------

    for (const auto &point : cloud->points)
    {
        int ix, iy, iz;

        if (!worldToGrid(
                point.x, point.y, point.z,
                ix, iy, iz))
        {
            continue;
        }

        occupied_voxels_.insert(
            gridToIndex(ix, iy, iz));
    }

    // ---------------------------------------------------------
    // 2. REGALY MUSIA BYT PLNE
    // ---------------------------------------------------------
    //
    // Point cloud vidi hlavne povrchy polic a medzi nimi mozu
    // vzniknut "volne" voxely.
    //
    // V simulacii sa vsak cely blok regalov povazuje
    // za pevnu prekazku.
    //
    // Preto tento cely objem explicitne vyplnime.
    //
    // x: 1.07  -> 7.57
    // y: 6.293 -> 11.885
    // z: 0.0   -> 3.76

    fillBox(
    1.15f, 7.49f,
    6.293f, 7.343f,
    0.0f, 3.76f);

    // Druhy rad regalov
    fillBox(
    1.15f, 7.49f,
    10.835f, 11.885f,
    0.0f, 3.76f);

    // ---------------------------------------------------------
    // 3. SAFETY INFLATION
    // ---------------------------------------------------------

    inflateObstacles();

    // Informacie o mape
    std::cout << "Mapa bola uspesne nacitana." << std::endl;
    std::cout << "Pocet bodov: "
              << cloud->size() << std::endl;

    std::cout << "\nBounding box:" << std::endl;

    std::cout << "MIN: x = " << min_point_.x
              << " y = " << min_point_.y
              << " z = " << min_point_.z
              << std::endl;

    std::cout << "MAX: x = " << max_point_.x
              << " y = " << max_point_.y
              << " z = " << max_point_.z
              << std::endl;

    return true;
}


// -------------------------------------------------------------
// Vyplnenie celeho kvadra ako prekazky
// -------------------------------------------------------------

void VoxelMap::fillBox(
    float x_min, float x_max,
    float y_min, float y_max,
    float z_min, float z_max)
{
    int ix_min, iy_min, iz_min;
    int ix_max, iy_max, iz_max;

    if (!worldToGrid(
            x_min, y_min, z_min,
            ix_min, iy_min, iz_min))
    {
        std::cerr << "fillBox: minimum je mimo mapy."
                  << std::endl;
        return;
    }

    if (!worldToGrid(
            x_max, y_max, z_max,
            ix_max, iy_max, iz_max))
    {
        std::cerr << "fillBox: maximum je mimo mapy."
                  << std::endl;
        return;
    }

    for (int iz = iz_min; iz <= iz_max; ++iz)
    {
        for (int iy = iy_min; iy <= iy_max; ++iy)
        {
            for (int ix = ix_min; ix <= ix_max; ++ix)
            {
                occupied_voxels_.insert(
                    gridToIndex(ix, iy, iz));
            }
        }
    }
}


// -------------------------------------------------------------
// Inflation prekazok
// -------------------------------------------------------------

void VoxelMap::inflateObstacles()
{
    // Povodne prekazky su automaticky blokovane
    inflated_voxels_ = occupied_voxels_;

    int radius_voxels =
        static_cast<int>(
            std::ceil(
                safety_radius_ / voxel_size_));

    const std::size_t layer_size =
        static_cast<std::size_t>(nx_) *
        static_cast<std::size_t>(ny_);

    for (std::size_t index : occupied_voxels_)
    {
        // Spatny prevod 1D indexu na ix, iy, iz
        int iz =
            static_cast<int>(
                index / layer_size);

        std::size_t remainder =
            index % layer_size;

        int iy =
            static_cast<int>(
                remainder /
                static_cast<std::size_t>(nx_));

        int ix =
            static_cast<int>(
                remainder %
                static_cast<std::size_t>(nx_));

        // Prehladavame okolie povodne obsadeneho voxelu
        for (int dz = -radius_voxels;
             dz <= radius_voxels;
             ++dz)
        {
            for (int dy = -radius_voxels;
                 dy <= radius_voxels;
                 ++dy)
            {
                for (int dx = -radius_voxels;
                     dx <= radius_voxels;
                     ++dx)
                {
                    int new_x = ix + dx;
                    int new_y = iy + dy;
                    int new_z = iz + dz;

                    // Kontrola hranic mapy
                    if (new_x < 0 || new_x >= nx_ ||
                        new_y < 0 || new_y >= ny_ ||
                        new_z < 0 || new_z >= nz_)
                    {
                        continue;
                    }

                    // Skutocna 3D vzdialenost od prekazky
                    float distance =
                        std::sqrt(
                            static_cast<float>(
                                dx * dx +
                                dy * dy +
                                dz * dz)) *
                        voxel_size_;

                    // Voxely v safety radius oznacime ako blokovane
                    if (distance <= safety_radius_)
                    {
                        inflated_voxels_.insert(
                            gridToIndex(
                                new_x,
                                new_y,
                                new_z));
                    }
                }
            }
        }
    }
}


// -------------------------------------------------------------
// Svetove suradnice -> voxelove suradnice
// -------------------------------------------------------------

bool VoxelMap::worldToGrid(
    float x, float y, float z,
    int &ix, int &iy, int &iz) const
{
    if (!isInside(x, y, z))
    {
        return false;
    }

    ix = static_cast<int>(
        std::floor(
            (x - min_point_.x) /
            voxel_size_));

    iy = static_cast<int>(
        std::floor(
            (y - min_point_.y) /
            voxel_size_));

    iz = static_cast<int>(
        std::floor(
            (z - min_point_.z) /
            voxel_size_));

    return true;
}


// -------------------------------------------------------------
// 3D voxel index -> jeden 1D index
// -------------------------------------------------------------

std::size_t VoxelMap::gridToIndex(
    int ix, int iy, int iz) const
{
    return static_cast<std::size_t>(ix) +
           static_cast<std::size_t>(nx_) *
               (static_cast<std::size_t>(iy) +
                static_cast<std::size_t>(ny_) *
                    static_cast<std::size_t>(iz));
}


// -------------------------------------------------------------
// Kontrola, ci bod lezi v mape
// -------------------------------------------------------------

bool VoxelMap::isInside(
    float x, float y, float z) const
{
    return x >= min_point_.x &&
           x <= max_point_.x &&
           y >= min_point_.y &&
           y <= max_point_.y &&
           z >= min_point_.z &&
           z <= max_point_.z;
}


// -------------------------------------------------------------
// Je tam priamo prekazka?
// -------------------------------------------------------------

bool VoxelMap::isOccupied(
    float x, float y, float z) const
{
    int ix, iy, iz;

    if (!worldToGrid(
            x, y, z,
            ix, iy, iz))
    {
        // Mimo mapy povazujeme za prekazku
        return true;
    }

    return occupied_voxels_.count(
               gridToIndex(ix, iy, iz)) >
           0;
}


// -------------------------------------------------------------
// Je bod blokovany po safety inflation?
// -------------------------------------------------------------

bool VoxelMap::isBlocked(
    float x, float y, float z) const
{
    int ix, iy, iz;

    if (!worldToGrid(
            x, y, z,
            ix, iy, iz))
    {
        return true;
    }

    return inflated_voxels_.count(
               gridToIndex(ix, iy, iz)) >
           0;
}


// -------------------------------------------------------------
// Gettery
// -------------------------------------------------------------

std::size_t VoxelMap::getOccupiedCount() const
{
    return occupied_voxels_.size();
}

std::size_t VoxelMap::getInflatedCount() const
{
    return inflated_voxels_.size();
}

std::size_t VoxelMap::getTotalVoxelCount() const
{
    return static_cast<std::size_t>(nx_) *
           static_cast<std::size_t>(ny_) *
           static_cast<std::size_t>(nz_);
}

int VoxelMap::getNx() const
{
    return nx_;
}

int VoxelMap::getNy() const
{
    return ny_;
}

int VoxelMap::getNz() const
{
    return nz_;
}

float VoxelMap::getVoxelSize() const
{
    return voxel_size_;
}

float VoxelMap::getSafetyRadius() const
{
    return safety_radius_;
}
bool VoxelMap::gridToWorld(
    int ix, int iy, int iz,
    float &x, float &y, float &z) const
{
    if (ix < 0 || ix >= nx_ ||
        iy < 0 || iy >= ny_ ||
        iz < 0 || iz >= nz_)
    {
        return false;
    }

    // Stred daneho voxelu
    x = min_point_.x +
        (static_cast<float>(ix) + 0.5f) * voxel_size_;

    y = min_point_.y +
        (static_cast<float>(iy) + 0.5f) * voxel_size_;

    z = min_point_.z +
        (static_cast<float>(iz) + 0.5f) * voxel_size_;

    return true;
}


bool VoxelMap::isBlockedGrid(
    int ix, int iy, int iz) const
{
    if (ix < 0 || ix >= nx_ ||
        iy < 0 || iy >= ny_ ||
        iz < 0 || iz >= nz_)
    {
        return true;
    }

    return inflated_voxels_.count(
               gridToIndex(ix, iy, iz)) >
           0;
}