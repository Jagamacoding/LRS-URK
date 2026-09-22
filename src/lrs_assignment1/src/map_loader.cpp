#include <iostream>
#include <string>

#include "lrs_assignment1/voxel_map.hpp"


int main(int argc, char **argv)
{
    if (argc != 2 && argc != 3 && argc != 6)
    {
        std::cerr << "Pouzitie:\n";
        std::cerr << "  map_loader <map.pcd>\n";
        std::cerr << "  map_loader <map.pcd> <safety_radius>\n";
        std::cerr << "  map_loader <map.pcd> <safety_radius> <x> <y> <z>\n";

        return 1;
    }

    // Velkost jedneho voxelu = 25 cm
    const float voxel_size = 0.25f;

    // Predvoleny safety radius
    float safety_radius = 0.5f;

    if (argc >= 3)
    {
        safety_radius = std::stof(argv[2]);
    }

    // Vytvorenie voxelovej mapy
    VoxelMap map(
        voxel_size,
        safety_radius);

    // Nacitanie PCD mapy
    if (!map.load(argv[1]))
    {
        return 1;
    }

    std::cout << "\nVoxel size: "
              << map.getVoxelSize()
              << " m"
              << std::endl;

    std::cout << "Safety radius: "
              << map.getSafetyRadius()
              << " m"
              << std::endl;

    std::cout << "Rozmery gridu: "
              << map.getNx() << " x "
              << map.getNy() << " x "
              << map.getNz()
              << std::endl;

    std::cout << "Celkovy pocet voxelov: "
              << map.getTotalVoxelCount()
              << std::endl;

    std::cout << "Pocet obsadenych voxelov pred inflation: "
              << map.getOccupiedCount()
              << std::endl;

    std::cout << "Pocet voxelov po inflation: "
              << map.getInflatedCount()
              << std::endl;

    // Ak sme zadali aj x y z, otestujeme konkretny bod
    if (argc == 6)
    {
        float x = std::stof(argv[3]);
        float y = std::stof(argv[4]);
        float z = std::stof(argv[5]);

        std::cout << "\nTest bodu: "
                  << x << ", "
                  << y << ", "
                  << z
                  << std::endl;

        if (!map.isInside(x, y, z))
        {
            std::cout << "MIMO MAPY"
                      << std::endl;
        }
        else if (map.isOccupied(x, y, z))
        {
            std::cout
                << "OBSADENE - prekazka"
                << std::endl;
        }
        else if (map.isBlocked(x, y, z))
        {
            std::cout
                << "BLOKOVANE - safety zona"
                << std::endl;
        }
        else
        {
            std::cout << "VOLNE"
                      << std::endl;
        }
    }

    return 0;
}