# Curious Robot

Gazebo Harmonic auf macOS: LiDAR-Perception, reaktive Navigation und
Weltpose/Pfadaufzeichnung und persistente besuchte Zellen (Session 4).
Die Localization verwendet die echte Simulatorpose;
SLAM, Hinderniskarten, Zielnavigation und ROS 2 sind nicht implementiert.

## Starten

Alle Befehle aus dem Repository-Root ausführen. Server und GUI getrennt starten.

Terminal 1 – Server:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/simulation/robot:$GZ_SIM_RESOURCE_PATH"
gz sim -s simulation/worlds/basic_world.sdf
```

Terminal 2 – GUI, anschließend **Run** drücken:

```bash
gz sim -g
```

Die lokale `.venv` ist für die installierten Homebrew-Gazebo-Bindings eingerichtet.
In jedem Python-Terminal aktivieren:

```bash
source .venv/bin/activate
```

Falls die Umgebung neu angelegt werden muss, mit dem zu den Gazebo-Bindings
passenden Homebrew-Python (hier 3.14):

```bash
/opt/homebrew/bin/python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install protobuf matplotlib
.venv/bin/python -c 'from gz.transport13 import Node; from gz.msgs10.pose_v_pb2 import Pose_V'
```

Geprüft mit Python 3.14.7, Gazebo Sim 8.15.0, Protobuf 7.36.2 und Matplotlib 3.11.2.
Die Gazebo-Bindings kommen aus der vorhandenen Homebrew-Installation, nicht von pip.
Ein anderer Python-Interpreter findet diese versionsgebundenen Bindings ggf. nicht.

Terminal 3 – Navigation:

```bash
source .venv/bin/activate
python -m src.navigation.navigation
```

Nur einen Controller starten, keine parallelen manuellen Fahrbefehle senden.
Ctrl+C sendet Stopp. Der Controller fährt mit 0,15 m/s und weicht vorne unter
1,0 m bzw. diagonal unter 0,6 m aus. Details stehen in `PROJECT_CONTEXT.md`.

## Pfad aufzeichnen und anzeigen

Terminal 4 – unabhängige Aufnahme, sendet keine Fahrbefehle:

```bash
source .venv/bin/activate
python -m src.localization.record --output recordings/run.csv --duration 60
```

Die Aufnahme läuft 60 Wanduhrsekunden; ohne `--duration` bis Ctrl+C.
Ohne `--output` entsteht ein automatisch benannter Pfad in `recordings/`.
Vorhandene CSV-Dateien werden nicht überschrieben: für jede Fahrt einen neuen
Namen wählen. Das Beenden der Aufnahme stoppt die Navigation nicht.

Danach:

```bash
python -m src.localization.plot recordings/run.csv
```

Oder als PNG ohne Fenster:

```bash
python -m src.localization.plot recordings/run.csv --output recordings/run.png --no-show
```

Grün markiert den Start, Rot das Ende; der Pfeil zeigt die letzte Blickrichtung.
Die Achsen zeigen Weltkoordinaten in Metern. Yaw ist intern Radiant: 0 zeigt nach
+X, +π/2 nach +Y. Gespeichert wird `timestamp,x,y,yaw`, standardmäßig alle
0,2 Simulationssekunden plus der letzte empfangene Punkt beim Beenden.
Während einer Pause entstehen bei gleichem Zeitstempel keine neuen Punkte.
Ein Rücksprung der Simulationszeit beendet die Aufnahme; danach neu starten.

Pose-Topic und Typ lassen sich prüfen mit:

```bash
gz topic -i -t /world/basic_world/dynamic_pose/info
gz topic -e -t /world/basic_world/dynamic_pose/info -n 1
```

Der vorhandene SceneBroadcaster liefert `gz.msgs.Pose_V`. Verwendet wird der
Eintrag `curious_robot`, dessen Pose in dieser Welt direkt im Weltrahmen liegt.
Die Aufzeichnung ist unabhängig von LiDAR-Auswertung und Navigation.

## Spatial Memory: besuchte Zellen

Parallel zur Navigation in einem eigenen Terminal starten:

```bash
source .venv/bin/activate
python -m src.memory.record
```

Der Prozess nutzt die bestehende Localization und sendet keine Fahrbefehle.
Alle zwei Sekunden erscheinen Zellanzahl und Coverage. Ctrl+C speichert den
aktuellen Stand in `data/spatial_memory.json`. Beim nächsten Start wird diese
Datei automatisch geladen; eine fehlende Datei bedeutet ein neues leeres Memory.
Die Navigation liest diese Datei nicht und fährt unverändert weiter.

Optional 60 Sekunden laufen lassen und JSON prüfen:

```bash
python -m src.memory.record --duration 60
python -m json.tool data/spatial_memory.json
# Erneut starten: Die zuvor gespeicherten Zellen bleiben vorhanden.
python -m src.memory.record --duration 10
```

Nur einen Memory-Prozess pro Datei starten. Für ein separates Experiment:

```bash
python -m src.memory.record --file data/experiment.json
```

Das Raster verwendet 0,5-m-Zellen und `floor(x / cell_size)` bzw.
`floor(y / cell_size)`, auch für negative Koordinaten. Die Weltinnenflächen liegen
bei ±4,9 m. Als halboffener Bereich `[-4.9, 4.9)` pro Achse schneidet das Rechteck
400 Rasterzellen; teilweise enthaltene Randzellen zählen mit.
Coverage ist `besuchte Zellen / 400`, ohne Abzug von Hindernissen oder Robotergröße.
Eine besuchte Zelle sagt daher nichts über ihre vollständige Befahrbarkeit aus.

JSON enthält Format/Version, Zellgröße, Grenzen und eindeutige Zellindizes.
Speicherung erfolgt bei Änderungen alle fünf Sekunden und final beim Beenden,
über eine temporäre Datei mit atomarem Ersetzen. Beschädigte Dateien oder
abweichende Konfigurationen werden mit einer Fehlermeldung abgewiesen.
`--cell-size`, `--bounds XMIN XMAX YMIN YMAX` und `--save-interval` sind optional;
für eine andere Rasterkonfiguration eine separate Datei verwenden.

Memory zählt empfangene Modellpositionen, keine Hindernisse oder sichtbaren
Flächen. Es interpoliert keine Empfangslücken. Grenzrauschen kann Nachbarzellen
markieren. Ein Simulationsreset behält die Besuche; für einen Neustart ohne alte
Besuche einen neuen Dateinamen wählen. Bei hartem Abbruch können die noch nicht
gespeicherten Besuche seit dem letzten Speicherintervall fehlen.

## Tests

Ohne laufendes Gazebo:

```bash
source .venv/bin/activate
python -m unittest discover -s tests -v
```

Nur Localization oder Navigation:

```bash
python -m unittest discover -s tests -p 'test_localization*.py' -v
python -m unittest discover -s tests -p 'test_navigation.py' -v
```

78 Tests; die zwei Plot-Tests werden ohne Matplotlib übersprungen. Die übrigen
Tests benötigen nur die Python-Standardbibliothek und simulieren Transportdaten.

Zur manuellen Prüfung: Pfad und Heading mit der GUI vergleichen, eine längere
Fahrt mit Ausweichmanövern aufnehmen und Pause/Resume sowie Ctrl+C der Navigation
prüfen. Enge Ecken und die gesamte Roboterfläche beim Drehen werden noch nicht
vollständig abgesichert. Der Python-Timeout kann einen hart beendeten Controller
nicht ersetzen; Gazebo behält den letzten Fahrbefehl bis zu einem neuen Befehl.
