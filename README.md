# LRS-URK — Zadanie 1

ROS 2 riešenie autonómnej misie dronu: spracovanie 3D mapy a plánovanie A* (A1.1),
let podľa CSV v hangári (A1.2) a vonkajšia misia s GPS, štvorcom a osmičkou (A1.3).
Vnútornú aj vonkajšiu misiu vykonáva spoločný riadiaci uzol.

## Dokumentácia a spustenie

- [Kompletný návod](src/lrs_assignment1/README.md): závislosti, samostatné spustenie
  A1.1, A1.2 a A1.3, záznamy, analýza a testy.
- [Technická dokumentácia](src/lrs_assignment1/docs/controller_design.md):
  algoritmy, parametre, súradnicové systémy a diagramy.
- [Overenie a výsledky](src/lrs_assignment1/docs/validation.md):
  merania, grafy, rozsah overenia a obmedzenia.
- [Overenie plánovača](src/lrs_assignment1/docs/a11_validation.md).
- [Záznamy letov](src/lrs_assignment1/recordings/README.md).

## Zostavenie

Predpokladá sa Ubuntu 24.04, ROS 2 Jazzy a nainštalované závislosti uvedené
v návode. Tento repozitár tvorí ROS workspace. V príkazoch je umiestnený
v `$HOME/LRS-URK-submission`; pri inom umiestnení treba cestu upraviť.
Učiteľský repozitár s mapami, modelmi a skriptmi je samostatne v `$HOME/LRS-URK`.

```bash
source /opt/ros/jazzy/setup.bash
cd "$HOME/LRS-URK-submission"
colcon build --packages-select lrs_assignment1 --executor sequential
source install/setup.bash
```

Základné automatické testy po zostavení:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/LRS-URK-submission/install/setup.bash"
cd "$HOME/LRS-URK-submission"
colcon test --packages-select lrs_assignment1 --event-handlers console_direct+
colcon test-result --verbose
```

## Obsah repozitára

```text
src/lrs_assignment1/
├── src/, include/      zdrojové kódy a hlavičky
├── config/             konfigurácie indoor a outdoor
├── missions/           CSV misie
├── scripts/            analýza rosbagov
├── test/               automatické testy
├── docs/               dokumentácia a výsledky
└── recordings/         komprimované záznamy simulácií
```

Priečinky `build`, `install`, `log` a rozbalené záznamy nie sú súčasťou zdrojov.
Výsledky sa vzťahujú na simuláciu Gazebo + ArduPilot SITL; rozsah ich platnosti
opisuje protokol overenia. Pred použitím na reálnom drone platia podmienky kurzu.
