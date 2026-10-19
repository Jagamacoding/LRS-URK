# Záznamy simulačných letov

Archívy obsahujú ROS správy z letov v Gazebe s ArduPilot SITL a MAVROS.
Nejde o videá ani o výstupy testu s náhradou MAVROS.

| Archív | Obsah | Priečinok po rozbalení |
|---|---|---|
| [indoor_success.tar.gz](indoor_success.tar.gz) | Dokončená indoor misia vrátane natočení, `landtakeoff` a konečného pristátia s vypnutím motorov. | `indoor_run2` |
| [outdoor_run1.tar.gz](outdoor_run1.tar.gz) | Prvý outdoor let: štvorec, osmička, návrat a pristátie. Obsahuje aj dlhšie čakanie pri štarte a všetky zaznamenávané ROS témy. | `outdoor_run1` |
| [outdoor_run2.tar.gz](outdoor_run2.tar.gz) | Druhý úspešný outdoor let za sebou; zaznamenaná vybraná telemetria a cieľové správy. | `outdoor_run2` |
| [outdoor_final.tar.gz](outdoor_final.tar.gz) | Kontrolný outdoor let z 5. 10. 2026. | `outdoor_final` |
| [outdoor_user_01.tar.gz](outdoor_user_01.tar.gz) | Samostatné opakovanie outdoor misie podľa návodu, dokončené 5. 10. 2026. | `outdoor_test_01` |

Pri kontrolnom behu `outdoor_final` boli uložené aj
[kontrolné súčty zdrojov a spustiteľného súboru](../docs/results/outdoor_final_source_manifest.json).
Metriky, grafy a CSV sú v [docs/results](../docs/results).
[Správa o overení](../docs/validation.md) vysvetľuje výsledky aj ich limity.

## Rozbalenie a analýza

Nasledujúce príkazy predpokladajú pracovný priestor `$HOME/LRS-URK-submission`.
Priečinok na rozbalenie musí byť nový alebo prázdny, aby sa neprepísal
predchádzajúci záznam.

```bash
source /opt/ros/jazzy/setup.bash
export LRS_PACKAGE="$HOME/LRS-URK-submission/src/lrs_assignment1"
mkdir -p "$HOME/LRS-URK-submission/replay_outdoor_user_01"
cd "$HOME/LRS-URK-submission/replay_outdoor_user_01"
tar -xzf "$LRS_PACKAGE/recordings/outdoor_user_01.tar.gz"
ros2 bag info outdoor_test_01

python3 "$LRS_PACKAGE/scripts/analyze_bag.py" outdoor_test_01 \
  --output outdoor_user_01_analysis
```

Vznikne graf `outdoor_user_01_analysis.png`, tabuľka vzoriek `.csv`
a číselný report `.json`. Pri inom archíve zmeňte názov archívu aj názov
rozbaleného priečinka podľa tabuľky. Na prehrávanie ani analýzu netreba
spúšťať Gazebo, SITL ani MAVROS.

## Prehrávanie polohy v RViz

V prvom termináli spustite RViz na oddelenej ROS doméne:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=49
rviz2
```

V druhom termináli spustite záznam:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=49
ros2 bag play "$HOME/LRS-URK-submission/replay_outdoor_user_01/outdoor_test_01" --loop
```

V RViz nastavte **Fixed Frame** na `map` a pridajte zobrazenie **Pose** pre
`/mavros/local_position/pose`. Ak pri téme vidíte nekompatibilitu QoS,
nastavte pri zobrazení spoľahlivosť na **Best Effort**. Šípka zobrazuje
zaznamenanú polohu a natočenie; model dronu ani prostredie Gazeba sa tým
neprehrávajú. Celú stopu letu naraz ukazuje vytvorený PNG graf.

Doména 49 oddeľuje prehrávanie od misie na doméne 47. Neprehrávajte cieľové
správy zo záznamu na doméne, ku ktorej je pripojený živý autopilot.
Prehrávanie ukončite cez **Ctrl+C**.
