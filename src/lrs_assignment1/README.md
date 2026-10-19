# Zadanie 1 — autonómna misia dronu

Balík `lrs_assignment1` obsahuje spracovanie 3D mapy a plánovač A* pre A1.1,
vykonávanie misie zo súboru v hangári pre A1.2 a vonkajšiu misiu s GPS, štvorcom
a osmičkou pre A1.3. Obe letové misie vykonáva uzol `mission_control`, spustený
programom `mission_node`. Stabilizáciu dronu a riadenie motorov zabezpečuje ArduPilot.

| Dokument | Obsah |
|---|---|
| [Technická dokumentácia](docs/controller_design.md) | Algoritmy, dôvody ich výberu, parametre, súradnicové systémy a dva diagramy riešenia. |
| [Overenie a výsledky](docs/validation.md) | Podmienky merania, výsledky simulácií, testy a známe obmedzenia. |
| [Overenie plánovača](docs/a11_validation.md) | Merania A1.1 a kontrola bezpečnosti naplánovaných trás. |
| [Záznamy letov](recordings/README.md) | Dostupné rosbagy, ich obsah a prehrávanie. |

Nasledujúce príkazy sú návodom na samostatné spustenie jednotlivých častí.
Testy A1.1 netreba spúšťať pred A1.2. A1.2 si mapu načíta a trasy naplánuje sama.
A1.3 mapu ani vnútornú CSV misiu nepoužíva.

## 1. Prostredie a zostavenie

Overené prostredie: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic a ArduPilot SITL.
Potrebné sú C++17, CMake, `colcon`, PCL, MAVROS s geografickými dátami a plugin
`ardupilot_gazebo`. Simulačný čas prenáša balík `ros_gz_bridge`. Analýza záznamov
používa Python 3, `rosbag2_py`, NumPy a Matplotlib; integračné testy tiež `rclpy`.
ROS závislosti balíka sú uvedené v [package.xml](package.xml).

Simulátor a jeho závislosti sa inštalujú podľa učiteľského repozitára
`LRS-URK/README.md`, časti *Installation*. Nasledujúci návod predpokladá túto
štruktúru; `$HOME` je domovský priečinok aktuálneho používateľa:

```text
$HOME/
├── LRS-URK/                         zadanie, mapy, modely a simulačné skripty
├── ardupilot/                       autopilot vrátane SITL
├── ardupilot_gazebo/build/           zostavený plugin Gazeba
└── LRS-URK-submission/
    └── src/lrs_assignment1/         tento ROS balík
```

Na inom počítači umiestnite tento balík do `src/lrs_assignment1` svojho ROS
workspace. Prenášajú sa zdroje, konfigurácie a misie; adresáre `build`, `install`
a `log` vzniknú lokálne zostavením. Ak používate iné cesty, upravte ich v príkazoch.
Učiteľský repozitár slúži ako zdroj mapy a simulátora; pri týchto krokoch sa jeho
zdrojové súbory neupravujú.

V Ubuntu termináli najskôr overte dostupnosť prostredia:

```bash
ls /opt/ros/jazzy/setup.bash
ls "$HOME/LRS-URK-submission/src/lrs_assignment1/package.xml"
ls "$HOME/LRS-URK/maps/FEI_LRS_PCD/map.pcd"
```

Ak cesta neexistuje, najprv opravte umiestnenie alebo inštaláciu. Potom zostavte balík:

```bash
source /opt/ros/jazzy/setup.bash
cd "$HOME/LRS-URK-submission"
colcon build --packages-select lrs_assignment1 --executor sequential
source install/setup.bash
ros2 pkg executables lrs_assignment1
```

Vo výpise majú byť okrem analytického skriptu programy `map_loader`, `planner_test`
a `mission_node`. Prvý `source` sprístupní ROS, druhý zostavený balík. Prostredie
treba načítať v každom novom termináli. Po zmene C++ kódu treba balík zostaviť znova;
zmena CSV alebo YAML používaného priamo zo `src/` vyžaduje iba nové spustenie uzla.

## 2. A1.1 — samostatné overenie mapy a plánovania

Táto časť nepotrebuje bežiaci Gazebo, SITL ani MAVROS. V jednom termináli:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/LRS-URK-submission/install/setup.bash"
export A11_MAP="$HOME/LRS-URK/maps/FEI_LRS_PCD/map.pcd"

# Mapa, rozmery mriežky a počty voxelov; polomer inflácie 0,5 m.
ros2 run lrs_assignment1 map_loader "$A11_MAP" 0.5

# Obsadenosť bodu x=13, y=7, z=1 m; očakávaný výsledok VOLNE.
ros2 run lrs_assignment1 map_loader "$A11_MAP" 0.5 13 7 1

# Prvá trasa: štart (13,7,1), cieľ (8.65,2.02,1).
ros2 run lrs_assignment1 planner_test "$A11_MAP" 0.5 13 7 1 8.65 2.02 1

# Druhá trasa obsahuje aj zmenu výšky.
ros2 run lrs_assignment1 planner_test "$A11_MAP" 0.5 8.65 2.02 1 4.84 5.37 2

# Porovnanie pri väčšom bezpečnostnom polomere.
ros2 run lrs_assignment1 planner_test "$A11_MAP" 0.65 13 7 1 8.65 2.02 1

# Neplatný štart mimo mapy: očakáva sa nenájdená cesta a návratový kód 2.
ros2 run lrs_assignment1 planner_test "$A11_MAP" 0.5 -10 -10 1 13 7 1
echo $?
```

Argumenty `planner_test` majú poradie `mapa polomer sx sy sz gx gy gz`.
Súradnice aj polomer sú v metroch. Program vypíše čas plánovania, počet bodov pred
a po zjednodušení a obe trasy. Samostatné nástroje majú voxel pevne nastavený na
0,25 m. Výpis `collision-free: ANO` pochádza zo vzorkovanej kontroly, ktorá má
známy nedostatok pri krátkych prienikoch; podrobnosti a nezávislá kontrola sú
v [protokole A1.1](docs/a11_validation.md).

## 3. A1.2 — samostatné spustenie v hangári

Začnite s dronom na východiskovom mieste a vypnutými motormi. Pri prechode z inej
misie ukončite jej simulátor a spustite nový. V každom z nasledujúcich terminálov
nechajte proces bežať. Gazebo, SITL a MAVROS sa podľa zadania spúšťajú ručne,
každý samostatne. Clock bridge, záznam a riadiaci uzol sú ďalšie procesy.

### Terminál 1 — Gazebo

```bash
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION=lrs_assignment1_demo
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
cd "$HOME/LRS-URK"
bash scripts/run_gazebo.sh
```

Otvorí sa hangár `fei_lrs_gazebo.world`. Počkajte na načítanie sveta a nechajte
simuláciu prehrávať. Skript nastaví cestu k modelom; premenná vyššie sprístupní
plugin prepájajúci fyziku Gazeba s autopilotom.

### Terminál 2 — ArduPilot SITL

```bash
source /opt/ros/jazzy/setup.bash
export HOME_LOCATION='48.15084570555732,17.072729745416016,150,0'
cd "$HOME/LRS-URK"
bash scripts/run_sitl.sh
```

Skript spustí simulovaný autopilot a MAVProxy. `gazebo-iris` určuje typ rámu,
`--model JSON` protokol spojenia s Gazebom a súbor parametrov nastavenie rámu
a motorov. Nevykonávajte ručne vzlet ani aktiváciu motorov; zabezpečí ich misia.

### Terminál 3 — MAVROS

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=47
export FCU_URL='udp://0.0.0.0:14550@'
cd "$HOME/LRS-URK"
bash scripts/run_mavros.sh
```

MAVROS prepája ROS témy a služby s MAVLink komunikáciou autopilota. Počkajte na
správu o nadviazaní spojenia. Port 14550 zodpovedá výstupu MAVProxy v skripte SITL.

### Terminál 4 — simulačný čas

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=47
export GZ_PARTITION=lrs_assignment1_demo
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

Bridge prenáša Gazebo čas do ROS `/clock`. S parametrom `use_sim_time:=true`
sa časový priebeh trajektórie prispôsobí rýchlosti simulácie. `GZ_PARTITION` musí
byť rovnaká v termináloch 1 a 4, `ROS_DOMAIN_ID` vo všetkých ROS procesoch tejto misie.

### Terminál 5 — záznam pred vzletom

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=47
mkdir -p "$HOME/LRS-URK-submission/my_recordings"
cd "$HOME/LRS-URK-submission/my_recordings"
ros2 bag record -a -o indoor_test_01
```

Pre ďalší pokus použite nový názov, napríklad `indoor_test_02`; výstupný priečinok
nesmie už existovať. Záznam nie je podmienkou riadenia, ale je výstupom zadania.

### Terminál 6 — vnútorná misia

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/LRS-URK-submission/install/setup.bash"
export ROS_DOMAIN_ID=47
cd "$HOME/LRS-URK-submission"
ros2 run lrs_assignment1 mission_node --ros-args \
  --params-file src/lrs_assignment1/config/indoor.yaml \
  -p map_file:="$HOME/LRS-URK/maps/FEI_LRS_PCD/map.pcd" \
  -p mission_file:="$HOME/LRS-URK-submission/src/lrs_assignment1/missions/indoor.csv" \
  -p use_sim_time:=true \
  -p telemetry_timeout:=15.0
```

Uzol načíta mapu, naplánuje a skontroluje všetkých šesť úsekov. Po splnení
vstupných kontrol automaticky prepne na `GUIDED`, aktivuje motory a vzlietne.
Nasledujú cieľové body, natočenia, medzipristátie s opätovným vzletom a záverečné
pristátie. Úspešný koniec je `State: DONE` po vypnutí motorov. Hlásenie
`using raw A* route` znamená použitie overenej trasy po odmietnutí skratky.
`ABORT` znamená prerušený, nie dokončený beh.

Polohy v CSV sú ciele; plánovač môže medzi nimi meniť aj výšku. Pri treťom úseku
dodanej misie a inflácii 0,65 m trasa vystúpi približne na 4,625 m nad mapovou
nulou, aby prekonala rad regálov. Samotné dosiahnutie `DONE` nepreukazuje všetky
odstupy; tie sa hodnotia zo záznamu.

### Iný súbor misie

Pri novom behu z východiskového stavu nahraďte posledné spustenie týmto príkazom
v rovnako pripravenom termináli 6:

```bash
ros2 run lrs_assignment1 mission_node --ros-args \
  --params-file "$HOME/LRS-URK-submission/src/lrs_assignment1/config/indoor.yaml" \
  -p map_file:="$HOME/LRS-URK/maps/FEI_LRS_PCD/map.pcd" \
  -p mission_file:="$HOME/LRS-URK-submission/src/lrs_assignment1/missions/indoor_alternative.csv" \
  -p use_sim_time:=true \
  -p telemetry_timeout:=15.0
```

Parameter `mission_file` môže obsahovať cestu k vlastnému CSV. Formát je
`x,y,z,precision,task`, desatinný oddeľovač je bodka, komentár začína `#`:

```csv
x,y,z,precision,task
13.00,7.00,1.00,soft,takeoff
8.65,2.02,1.00,soft,-
4.84,5.37,2.00,hard,yaw180
2.08,9.74,1.75,hard,-
8.84,6.90,2.00,hard,landtakeoff
2.81,8.15,1.50,soft,yaw90
13.60,1.50,2.00,hard,land
```

Prvý riadok musí byť `takeoff` nad miestom štartu, posledný `land`.
Medzipristátie sa zapisuje `landtakeoff`; `-` znamená bod bez ďalšej úlohy.
`yaw90` aj `yaw 90` sú platné. Uhol je absolútny smer v mapovom rámci, v stupňoch,
kladne proti smeru hodinových ručičiek. Úlohy vyžadujúce držanie polohy sa ustália
aj na riadku označenom `soft`.

Konfigurácia počíta so štartom v mape `(13,7,0)` a rotáciou mapy do lokálneho ENU
o `pi/2`. Pri inom štarte treba zmeniť `map_origin_x/y/z`, prvý riadok CSV a podľa
sveta aj `map_to_local_yaw`. Pristávacie miesta musia mať voľný zvislý prístup;
program nepodporuje ľubovoľné pristávanie na policiach či strechách.

## 4. A1.3 — samostatné spustenie vonku

Predchádzajúci vnútorný let nemusí byť spustený ani vykonaný. Potrebný je zostavený
balík z časti 1 a čerstvý vonkajší simulátor. Pri zmene sveta najskôr ukončite
všetky procesy predošlého behu po pristátí a vypnutí motorov.

### Terminál 1 — vonkajší svet

```bash
source /opt/ros/jazzy/setup.bash
export GZ_PARTITION=lrs_assignment1_demo
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
cd "$HOME/LRS-URK"
bash scripts/run_gazebo.sh fei_lrs_outdoor.world
```

### Terminál 2 — autopilot pre vonkajší svet

```bash
source /opt/ros/jazzy/setup.bash
export HOME_LOCATION='48.15144444,17.07375000,150,0'
cd "$HOME/LRS-URK"
bash scripts/run_sitl.sh
```

`HOME_LOCATION` nastavuje geografické umiestnenie simulačného sveta podľa jeho
SDF súboru. Misia si počiatok zachytí zo senzorov pri štarte; tieto absolútne
súradnice nie sú cieľmi zabudovanými do riadiaceho programu.

### Terminál 3 — MAVROS

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=47
export FCU_URL='udp://0.0.0.0:14550@'
cd "$HOME/LRS-URK"
bash scripts/run_mavros.sh
```

### Terminál 4 — simulačný čas

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=47
export GZ_PARTITION=lrs_assignment1_demo
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

### Terminál 5 — záznam pred vzletom

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=47
mkdir -p "$HOME/LRS-URK-submission/my_recordings"
cd "$HOME/LRS-URK-submission/my_recordings"
ros2 bag record -a -o outdoor_test_01
```

Ak už `outdoor_test_01` existuje, zvoľte nový názov, napríklad `outdoor_test_02`,
a použite ho aj pri následnej analýze. Počas záznamu nechajte tento terminál bežať.

### Terminál 6 — vonkajšia misia

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/LRS-URK-submission/install/setup.bash"
export ROS_DOMAIN_ID=47
cd "$HOME/LRS-URK-submission"
ros2 run lrs_assignment1 mission_node --ros-args \
  --params-file src/lrs_assignment1/config/outdoor.yaml \
  -p use_sim_time:=true \
  -p telemetry_timeout:=15.0
```

Program počká na použiteľné GPS s aspoň ôsmimi satelitmi, čerstvú telemetriu
a zhodu kompasu s lokálnym smerom. Vzlietne 5 m nad štart, preletí uzavretý
štvorec so stranou 5 m a nosom smerujúcim na POI, presunie sa do stredu a vykoná
osmičku 6 × 3 m s nosom v smere pohybu. Potom sa vráti na štart a pristane.
Počiatočný POI je na mieste vzletu; nie je samostatným viditeľným objektom v Gazebe.
Jeden okruh osmičky pri `omega=0.2` trvá približne 31,4 s simulačného času.

Tu sa nepoužíva PCD ani CSV. Geometria, POI a rýchlosť sú parametre v
[outdoor.yaml](config/outdoor.yaml). Pri skúške pomalšej osmičky možno do príkazu
pridať `-p omega:=0.1`; pevné rozmery požadované zadaním sa nemenia. Parametre
riadenia sa načítajú pri štarte, preto sa menia pred novým behom uzla.

Tieto príkazy sú pre SITL. Let na reálnom drone vyžaduje podmienky kurzu,
konzultáciu, prítomnosť učiteľa a bezpečnostného pilota. Pred ním zadanie vyžaduje
aj dva úplné simulačné behy za sebou a preukázané prerušenie misie.

## 5. Sledovanie, prerušenie a ukončenie

V ďalšom termináli možno sledovať stav; `echo` ukončíte cez Ctrl+C:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=47
ros2 topic echo /mission_control/state
```

Na prerušenie práve prebiehajúcej misie:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=47
ros2 service call /mission_control/abort std_srvs/srv/Trigger '{}'
```

Služba zastaví publikovanie cieľov a pri dostupnom spojení, aktívnych motoroch
a režime `GUIDED` požiada o `LAND`. Pri strate spojenia alebo zmene režimu pilotom
program prestane posielať ciele; automatické pristátie vtedy nemožno zaručiť.
Uzol v stave `ABORT` sa sám nevráti do misie a opakované volanie abortu už
nevyvolá ďalší prechod. Ak sa spojenie po výpadku obnovilo a simulovaný dron
ostal visieť, možno priamo požiadať autopilota o pristátie:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=47
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'LAND'}"
```

Úspešná odpoveď služby ešte nie je potvrdením pristátia; sledujte Gazebo a stav
motorov cez `/mavros/state`. Po úspešnej misii očakávajte `DONE`. Recorder nechajte
bežať ešte niekoľko sekúnd po vypnutí motorov, potom ho ukončite cez Ctrl+C, aby
zapísal metadáta. Následne ukončite riadiaci uzol, bridge, MAVROS, SITL a Gazebo.
Samotné Ctrl+C v riadiacom uzle nie je príkazom na pristátie.

## 6. Vyhodnotenie a prehrávanie záznamu

Po ukončení recordera vytvorte graf a číselný súhrn. Príklad pre vlastný vonkajší beh:

```bash
source /opt/ros/jazzy/setup.bash
cd "$HOME/LRS-URK-submission/src/lrs_assignment1"
python3 scripts/analyze_bag.py \
  "$HOME/LRS-URK-submission/my_recordings/outdoor_test_01" \
  --output "$HOME/LRS-URK-submission/my_recordings/outdoor_test_01_analysis"
```

Vzniknú `.png`, `.json` a `.csv`. PNG otvoríte vo VS Code. Zobrazuje požadovanú
a nameranú dráhu, výšku a chybu smeru. V JSON skontrolujte `complete: true`,
`final_armed: false` a posledný stav `DONE`. Pri inom POI pridajte analytickému
skriptu zodpovedajúce `--poi-east` a `--poi-north`. Pre vnútorný let nahraďte
vstupný priečinok a výstupný názov hodnotou `indoor_test_01`.

Rosbag obsahuje ROS správy, nie automaticky video okna Gazeba. Na prehrávanie
pohybu v RViz použite inú ROS doménu než pre bežiaci simulátor. V prvom termináli:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=49
rviz2
```

Nastavte `Fixed Frame` na `map` a pridajte zobrazenie `Pose` pre tému
`/mavros/local_position/pose`. Pri hlásení nekompatibility QoS nastavte
spoľahlivosť zobrazenia na `Best Effort`. Potom v druhom termináli:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=49
ros2 bag play "$HOME/LRS-URK-submission/my_recordings/outdoor_test_01"
```

Šípka ukazuje polohu a natočenie zo záznamu; nejde o nové riadenie Gazeba.
Záznam môže začínať čakaním pred vzletom. Na opakovanie pridajte `--loop`.
Dodané archívy sa rozbaľujú podľa [prehľadu záznamov](recordings/README.md).

## 7. Samostatné automatické testy

Základné testy matematiky, CSV parsera a kontroly úsekov nepotrebujú simulátor:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/LRS-URK-submission/install/setup.bash"
cd "$HOME/LRS-URK-submission"
colcon test --packages-select lrs_assignment1 --event-handlers console_direct+
colcon test-result --verbose
```

Test geometrickej kontroly aj s konkrétnou regresiou z mapy hangára:

```bash
source /opt/ros/jazzy/setup.bash
cd "$HOME/LRS-URK-submission"
./build/lrs_assignment1/path_guard_tests "$HOME/LRS-URK/maps/FEI_LRS_PCD/map.pcd"
```

Integračné testy používajú náhradu MAVROS a spúšťajú skutočný `mission_node`.
Kontrolujú jedenásť scenárov vrátane zamietnutých požiadaviek a výpadkov;
nenahrádzajú fyzikálnu simuláciu. Na doméne 48 nesmie byť iný MAVROS ani vozidlo:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/LRS-URK-submission/install/setup.bash"
export ROS_DOMAIN_ID=48
cd "$HOME/LRS-URK-submission/src/lrs_assignment1"
python3 test/mock_mavros_integration.py \
  --executable "$HOME/LRS-URK-submission/install/lrs_assignment1/lib/lrs_assignment1/mission_node" \
  --output "$HOME/LRS-URK-submission/my_recordings/mock_check"
```

Tieto testy sa pri spustení misie automaticky nevykonávajú.

## 8. Bežné problémy

| Príznak | Čo skontrolovať |
|---|---|
| `No such file or directory` pri `source` | Ubuntu terminál, nainštalovanú verziu ROS a skutočnú cestu workspace. Príkazy sú pre Jazzy. |
| Balík sa nenašiel | Úspešné zostavenie a `source "$HOME/LRS-URK-submission/install/setup.bash"` v danom termináli. |
| Gazebo nenačíta ArduPilot plugin | Cestu `GZ_SIM_SYSTEM_PLUGIN_PATH` a zhodu verzie pluginu s Gazebo Harmonic. |
| `Waiting for telemetry` | Bežiaci a nepozastavený Gazebo, SITL, MAVROS, správnu doménu a pripojenie autopilota. Outdoor navyše čaká na GPS. |
| Motory sa neaktivujú | Dôvod odmietnutia v konzole ArduPilota a výsledky služieb v logu misie. |
| `No safe route` alebo neplatný prvý bod | Mapu, CSV, bezpečnostný polomer a zhodu miesta štartu s transformáciou mapy. |
| Zastavený pohyb pri `use_sim_time:=true` | Bridge `/clock`, zhodnú `GZ_PARTITION` a bežiaci čas v Gazebe. |
| `MAVROS telemetry stale/disconnected` | Prichádzanie stavových a polohových správ; pri pomalej simulácii aj ich interval v reálnom čase. |

Príkazy na simuláciu používajú `telemetry_timeout:=15.0`. Je to explicitné
nastavenie pre pomalý beh s grafikou: v zaznamenanom prípade heartbeat prišiel
až po 6,35 s, čím prekročil limit 6 s. V `indoor.yaml` zostáva 6 s; bez konfigurácie
má uzol limit 3 s, ktorý platí aj pre samotný `outdoor.yaml`. Vyšší limit oneskoruje
aj rozpoznanie skutočného výpadku. Pri opakovanej chybe treba odmerať príčinu,
nie ďalej zvyšovať limit. Pozastavenie Gazeba môže vyvolať timeout, pretože
kontrola čerstvosti správ používa monotónny reálny čas.

## 9. Podklady zadania

Požiadavky sú v učiteľskom repozitári v `assignments/assignment1/`:
`01_map_and_path_planning.md`, `02_indoor_mission_execution.md`,
`03_outdoor_gps_mission.md` a `04_documentation_and_defence.md`.
Spúšťanie simulátora opisuje jeho `README.md`, ROS základy
`tutorial/ros2_cheatsheet.md` a callback skupiny príklad `mt_executor_demo`.
Hlavná technická dokumentácia tohto riešenia je v
[docs/controller_design.md](docs/controller_design.md).
