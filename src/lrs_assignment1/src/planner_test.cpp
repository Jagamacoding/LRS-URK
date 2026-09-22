#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "lrs_assignment1/astar_planner.hpp"
#include "lrs_assignment1/voxel_map.hpp"


int main(int argc, char **argv)
{
    if (argc != 9)
    {
        std::cerr
            << "Pouzitie:\n"
            << "planner_test <map.pcd> <safety_radius> "
            << "<start_x> <start_y> <start_z> "
            << "<goal_x> <goal_y> <goal_z>\n";

        return 1;
    }

    std::string map_path =
        argv[1];

    float safety_radius =
        std::stof(argv[2]);

    Point3D start{
        std::stof(argv[3]),
        std::stof(argv[4]),
        std::stof(argv[5])
    };

    Point3D goal{
        std::stof(argv[6]),
        std::stof(argv[7]),
        std::stof(argv[8])
    };

    const float voxel_size =
        0.25f;

    VoxelMap map(
        voxel_size,
        safety_radius);

    if (!map.load(map_path))
    {
        return 1;
    }

    std::cout
        << "\nSTART: "
        << start.x << ", "
        << start.y << ", "
        << start.z
        << std::endl;

    std::cout
        << "GOAL:  "
        << goal.x << ", "
        << goal.y << ", "
        << goal.z
        << std::endl;

    AStarPlanner planner(map);

    // --------------------------------------------------------
    // A*
    // --------------------------------------------------------

    auto time_start =
        std::chrono::steady_clock::now();

    std::vector<Point3D> path =
        planner.plan(
            start,
            goal);

    auto time_end =
        std::chrono::steady_clock::now();

    double planning_ms =
        std::chrono::duration<double, std::milli>(
            time_end - time_start)
            .count();

    std::cout
        << "\nPlanning time: "
        << std::fixed
        << std::setprecision(3)
        << planning_ms
        << " ms"
        << std::endl;

    if (path.empty())
    {
        std::cout
            << "Cesta sa nenasla."
            << std::endl;

        return 2;
    }

    // --------------------------------------------------------
    // Path simplification
    // --------------------------------------------------------

    std::vector<Point3D> simplified =
        planner.simplifyPath(path);

    bool safe =
        planner.isPathCollisionFree(
            simplified);

    std::cout
        << "\nPocet bodov pred zjednodusenim: "
        << path.size()
        << std::endl;

    std::cout
        << "Pocet bodov po zjednoduseni: "
        << simplified.size()
        << std::endl;

    std::cout
        << "Zjednodusena cesta collision-free: "
        << (safe ? "ANO" : "NIE")
        << std::endl;

    // --------------------------------------------------------
    // Raw A* cesta
    // --------------------------------------------------------

    std::cout
        << "\nRAW PATH:"
        << std::endl;

    for (std::size_t i = 0;
         i < path.size();
         ++i)
    {
        std::cout
            << i
            << ": "
            << path[i].x
            << ", "
            << path[i].y
            << ", "
            << path[i].z
            << std::endl;
    }

    // --------------------------------------------------------
    // Zjednodusena cesta
    // --------------------------------------------------------

    std::cout
        << "\nSIMPLIFIED PATH:"
        << std::endl;

    for (std::size_t i = 0;
         i < simplified.size();
         ++i)
    {
        std::cout
            << i
            << ": "
            << simplified[i].x
            << ", "
            << simplified[i].y
            << ", "
            << simplified[i].z
            << std::endl;
    }

    if (!safe)
    {
        return 3;
    }

    return 0;
}