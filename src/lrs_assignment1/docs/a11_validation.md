# A1.1 – overenie mapy a plánovača

Tento doplnkový protokol uvádza merania a hraničné prípady plánovania.
Algoritmy a ich zdôvodnenie opisuje [návrh riešenia](controller_design.md);
spustenie samostatných programov je v [README](../README.md).
Overenie prebehlo 3. októbra 2026 v zostavení Release s ROS 2 Jazzy a PCL.
Súhrn jednotlivých trás je uložený v [a11_mission_results.json](a11_mission_results.json).
Dočasné súbory prvého zostavenia ani všetky jeho podrobné logy sa nezachovali.

## Mapa a merania trás

Použitá učiteľská mapa `maps/FEI_LRS_PCD/map.pcd` má 620 311 bodov. Ohraničenie
je približne `(-0,30; -1,15; 0)` až `(17,95; 13,20; 5,9504)` m.

| Vlastnosť | Hodnota |
|---|---:|
| Hrana voxelu | 0,25 m |
| Počet buniek mriežky | 101 616 |
| Obsadené bunky vrátane vyplnených regálov | 24 293 |
| Blokované bunky pri inflácii 0,50 m | 60 015 |
| Blokované bunky pri inflácii 0,65 m | 63 465 |

Overilo sa všetkých šesť úsekov misií `exam_example.csv` a `mission_3_all.csv`
pri oboch polomeroch, spolu **24 plánovaní**. Všetky našli cestu.
A\* trvalo 0,961–31,995 ms; úplné cesty mali 9–49 bodov a návrhy zjednodušených
ciest 2–5 bodov. Časy nezahŕňajú načítanie mapy a závisia od zaťaženia počítača.

Príklad prvých troch úsekov `exam_example` pri polomere 0,65 m:

| Úsek | Body A\* | Body po zjednodušení | Čas A\* |
|---|---:|---:|---:|
| 1 | 21 | 2 | 1,085 ms |
| 2 | 21 | 3 | 3,784 ms |
| 3 | 30 | 5 | 31,995 ms |

Počet bodov nie je dôkaz bezpečnosti. Tretí zjednodušený úsek v tabuľke
konzervatívna kontrola A1.2 odmietne a let použije overenú úplnú trasu.
Počty v logu `mission_node` sa môžu líšiť od samostatného `planner_test`,
pretože pred validáciou dopĺňa presný štart a cieľ aj ako opakované body.

## Zistenie pri kontrole skratiek

Kontrola viditeľnosti v `AStarPlanner` vzorkuje úsečku s krokom najviac štvrtiny
voxelu. Aj tak môže vynechať krátky prienik cez roh bunky. Nezávislé overenie
rozdeľuje úsečku pri každej stene mriežky a skúma každý vzniknutý interval.
Reprodukovateľný prípad je piaty úsek `exam_example` pri inflácii 0,5 m:

- Štart `(8,84; 6,90; 2)` a cieľ `(2,81; 8,15; 1,5)` majú bezpečnú úplnú
  cestu s 26 bodmi.
- Zjednodušenie navrhne tri body. Spojnica `(8,84000015; 6,9000001; 2)` →
  `(7,57499981; 8,22500038; 1,625)` však prejde blokovaným voxelom.
- Bod prieniku je približne `(8,18584919; 7,58517838; 1,80608165)`.
  Leží v inflácii, nie priamo v obsadenom povrchu. Je to porušenie plánovacej
  rezervy, nie dôkaz fyzického nárazu.
- Metóda `isPathCollisionFree` túto skratku označí ako voľnú. Jej výsledok
  teda sám osebe nestačí na potvrdenie bezpečnosti.

Pri polomere 0,65 m prešlo všetkých 12 zjednodušených trás kontrolou intervalov
s nenulovou dĺžkou. Prísnejší `FlightPathGuard` navyše preveruje dotyky stien,
hrán a rohov; odmietol tretí úsek `exam_example`. **Všetkých 24 úplných ciest
prešlo aj touto konzervatívnou kontrolou.**

## Kontrola pred letom A1.2

[flight_path_guard.hpp](../include/lrs_assignment1/flight_path_guard.hpp)
preveruje úsečku po všetkých prekročeniach voxelových hraníc. Na hranici
skúma každú susednú bunku; pri 0,25 m voxeli používa numerickú toleranciu
približne 2,5 μm. Nekonečné súradnice, NaN a body mimo mapy odmieta.

```cpp
mission::FlightPathGuard guard(loaded_map);
bool safe_segment = guard.segmentFree(start, goal);
bool safe_route = guard.pathFree(path);
```

Riadiaci uzol doplní presné koncové body, preverí zjednodušenú cestu a pri
neúspechu preverí úplný výstup A\*. Ak nevyhovuje ani ten, skončí pred letom.
Rovnaká kontrola chráni spojenia pri preskakovaní medzibodov počas navigácie.
Nevýhodou je možnosť odmietnutia hraničnej, teoreticky tečnej trasy; prínosom
je odstránenie závislosti od vzdialenosti vzoriek.

## Hraničné prípady a rozsah dôkazov

| Prípad | Pozorovanie |
|---|---|
| Štart `(13; 7; 1)` a cieľ `(13,01; 7,01; 1,01)` v jednom voxeli | Plánovač vráti len cieľ. A1.2 dopĺňa štart, aby overila aj túto spojnicu. |
| Štart mimo mapy alebo cieľ v regáli | Prázdna cesta, bez pádu programu. |
| Syntetická nepriechodná stena | Vyčerpanie prehľadávania, výsledok „cesta neexistuje“, 33,901 ms a návratový kód 2. Dočasný vstup a podrobný log sa nezachovali. |
| Veľmi krátky prienik, nulová dĺžka, dotyk steny/hrany/rohu | Pokryté testami `path_guard_tests`. |

[Testy guardu](../test/path_guard_tests.cpp) zahŕňajú aj vyššie uvedenú regresiu
na učiteľskej PCD mape, ak sa jej cesta zadá ako argument. Opakovaný beh
5. októbra 2026 skončil návratovým kódom 0; uložený je
[výpis kontroly](results/recovery_checks.log). Matematické skúšky plánovača
a guardu nepreukazujú úspešný let simulátora ani skutočnú letovú rezervu.

Aktuálny svet hangára používa model `fei_lrs_racks` s osobitnými kolíznymi
kvádrami políc a stĺpikov. Text zadania opisuje aj starší model s jedným
plným objemom. Mapová reprezentácia vypĺňa dva celé rady regálov a zachováva
uličku, takže plánovanie nepoužíva medzery medzi policami. Pri zmene sveta
treba znovu porovnať PCD, vyplnené kvádre a skutočnú kolíznu geometriu.
