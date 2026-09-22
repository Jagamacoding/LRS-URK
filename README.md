# LRS-URK – Assignment 1

Riešenie Assignment 1 pre predmet LRS-URK.

## Structure

- `src/lrs_assignment1` – ROS 2 package
- `missions` – mission files
- `docs` – documentation
- `recordings` – simulation evidence / links

## A1.1 – 3D map and path planning

Aktuálne implementované:

- načítanie PCD mapy pomocou PCL
- 3D voxel grid s rozlíšením 0.25 m
- occupancy queries
- plná reprezentácia regálov
- nastaviteľný 3D obstacle inflation
- 3D A* planner s 26 susedmi
- detekcia prípadu, keď cesta neexistuje
- meranie času plánovania
- line-of-sight path simplification
- kontrola collision-free simplified path

## Build

```bash
cd ~/LRS-URK-submission
colcon build
source install/setup.bash
