# ROS2 LLM Agent – TurtleBot3 Patrol mit KI-Chatbot

Ein ROS2-Projekt, das einen TurtleBot3 in einer Gazebo-Simulation autonom auf Patrol schickt und gleichzeitig einen LLM-basierten Chatbot bereitstellt, der über ROS2-Tools mit dem Roboter interagieren kann.

---

## Projektübersicht

```
┌─────────────────────────────────────────────────────────┐
│                     Docker Container                    │
│                                                         │
│  ┌──────────────┐    ┌──────────────┐    ┌───────────┐ │
│  │   Gazebo +   │    │  Waypoint    │    │  LLM      │ │
│  │  TurtleBot3  │◄───│  Patrol      │    │  Agent    │ │
│  │  + Nav2      │    │  (Nav2)      │    │  Node     │ │
│  └──────┬───────┘    └──────────────┘    └─────┬─────┘ │
│         │ /odom, /scan, /tf                     │       │
│         └──────────────────────────────────────►│       │
│                                                 │       │
│                                       ┌─────────▼─────┐ │
│                                       │  LLM Chatbot  │ │
│                                       │  (Gemini API) │ │
│                                       └───────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### Pakete

| Paket | Beschreibung |
|-------|-------------|
| `turtlebot3_full_bringup` | Startet Gazebo, Nav2, RViz und den TurtleBot3 |
| `waypoint_patrol` | Fährt autonom 4 Wegpunkte in einer Endlosschleife ab |
| `llm_agent` | ROS2-Node mit 4 Service-Tools + CLI-Chatbot mit Gemini |

---

## Voraussetzungen

- **Ubuntu 22.04** (nativ oder WSL2)
- **Docker** & **Docker Compose**
- **Google Gemini API Key** (kostenlos unter [aistudio.google.com](https://aistudio.google.com))
- X11-Server für GUI (Gazebo/RViz) – unter Windows z.B. VcXsrv oder X410

---

## Schnellstart (empfohlen)

```bash
# Repository klonen
git clone <repo-url>
cd "llm agent - MRK/docker"

# API-Key setzen
export GOOGLE_API_KEY="dein-api-key-hier"

# Automatisches Setup & Start (Build + Docker)
chmod +x setup_and_run.sh
./setup_and_run.sh
```

Das Skript:
1. Aktiviert Docker
2. Erkennt automatisch GPU (NVIDIA) oder fällt auf CPU zurück
3. Fragt interaktiv nach den API-Keys
4. Baut das Docker-Image
5. Startet den Container

---

## Manueller Start (Schritt für Schritt)

### 1. Docker Image bauen

```bash
cd "llm agent - MRK/docker"
export GOOGLE_API_KEY="dein-api-key-hier"
docker compose build
```

### 2. Container starten

```bash
docker compose up -d
```

### 3. In den Container einsteigen

```bash
docker exec -it ros2_tb3_container bash
```

---

## ROS2-Workspace bauen (im Container)

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Simulation starten

### Terminal 1 – Gazebo + Nav2 + RViz

```bash
export TURTLEBOT3_MODEL=burger
source ~/ros2_ws/install/setup.bash
ros2 launch turtlebot3_full_bringup full_bringup.launch.py
```

Startet:
- Gazebo mit der `playground.world`
- TurtleBot3 Burger an Position `(-2.0, -0.5)`
- Nav2 mit vorkonfigurierter Karte (`playground_map_hq.yaml`)
- RViz zur Visualisierung

### Terminal 2 – Waypoint Patrol

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch waypoint_patrol patrol.launch.py
```

Der Roboter fährt automatisch die 4 Wegpunkte in einer Endlosschleife ab:

| Wegpunkt | x | y | yaw |
|----------|------|-------|-----|
| Home | 0.00 | 0.00 | 0° |
| A | -2.00 | 1.00 | 0° |
| B | -0.50 | -2.00 | 0° |
| Ziel | -2.00 | -1.50 | 0° |

### Terminal 3 – LLM Agent Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch llm_agent llm_agent.launch.py
```

Registriert die ROS2-Services, die der Chatbot als Tools nutzt.

### Terminal 4 – LLM Chatbot starten

```bash
source ~/ros2_ws/install/setup.bash
ros2 run llm_agent chat
```

---

## LLM Agent Tools

Der Chatbot hat Zugriff auf 4 ROS2-Services (Tools):

| Tool | ROS2-Service | Beschreibung |
|------|-------------|-------------|
| `get_robot_pose()` | `/llm_tools/get_robot_pose` | Aktuelle Position (x, y, yaw) im Map-Frame |
| `get_waypoints()` | `/llm_tools/get_waypoints` | Alle 4 Patrol-Wegpunkte mit Namen und Koordinaten |
| `calculate_distance(point_a, point_b)` | *(lokal)* | Euklidischer Abstand zwischen zwei 2D-Punkten |
| `get_robot_state()` | `/llm_tools/get_robot_state` | Vollständiger Sensor-Snapshot (Pose, Odom, Laser, IMU) |

### Beispiel-Fragen an den Chatbot

```
You> Where is the nearest waypoint?
You> Welcher Waypoint ist am weitesten entfernt?
You> How far are all waypoints from me?
You> What obstacles are near the robot?
You> Wie ist der aktuelle Roboterstatus?
```

### Token-Statistiken

Nach jeder Antwort werden Token-Verbrauch und Tool-Calls angezeigt:

```
[Stats] Tool calls: 3 | Tokens: 847 (prompt: 712, completion: 135)
```

---

## ROS2-Services manuell aufrufen

```bash
# Aktuelle Pose abfragen
ros2 service call /llm_tools/get_robot_pose std_srvs/srv/Trigger {}

# Alle Wegpunkte abfragen
ros2 service call /llm_tools/get_waypoints std_srvs/srv/Trigger {}

# Vollständigen Sensor-Status abfragen
ros2 service call /llm_tools/get_robot_state std_srvs/srv/Trigger {}

# Nächsten Wegpunkt berechnen
ros2 service call /llm_tools/get_nearest_waypoint std_srvs/srv/Trigger {}
```

---

## Topics überwachen

```bash
# Alle aktiven Topics anzeigen
ros2 topic list

# Odometrie-Daten
ros2 topic echo /odom

# Laser-Scan
ros2 topic echo /scan

# TF-Baum anzeigen
ros2 run tf2_tools view_frames
```

---

## Wegpunkte anpassen

Wegpunkte in `ros2_ws/src/waypoint_patrol/config/waypoints.yaml` bearbeiten:

```yaml
frame_id: map
waypoints:
  - {name: Home, x:  0.00, y:  0.00, yaw: 0.00}
  - {name: A,    x: -2.00, y:  1.00, yaw: 0.00}
  - {name: B,    x: -0.50, y: -2.00, yaw: 0.00}
  - {name: Ziel, x: -2.00, y: -1.50, yaw: 0.00}
```

Nach Änderungen neu bauen:

```bash
cd ~/ros2_ws && colcon build --symlink-install
```

---

## Projektstruktur

```
llm agent - MRK/
├── docker/
│   ├── Dockerfile              # ROS2 Humble + alle Abhängigkeiten
│   ├── docker-compose.yaml     # Container-Konfiguration
│   ├── setup_and_run.sh        # Automatisches Setup-Skript
│   ├── requirements.txt        # Python-Pakete (LangChain, Gemini, ...)
│   └── apt-packages.txt        # System-Pakete
└── ros2_ws/src/
    ├── turtlebot3_full_bringup/
    │   ├── launch/full_bringup.launch.py   # Haupt-Launchfile
    │   ├── maps/playground_map_hq.yaml     # Karte für Nav2
    │   ├── worlds/playground.world         # Gazebo-Welt
    │   └── config/init_nav2_params.yaml    # Nav2-Parameter
    ├── waypoint_patrol/
    │   ├── waypoint_patrol/patrol.py       # Patrol-Node
    │   ├── launch/patrol.launch.py
    │   └── config/waypoints.yaml           # Wegpunkt-Definitionen
    └── llm_agent/
        ├── llm_agent/
        │   ├── node.py         # ROS2-Node mit 4 Services
        │   ├── chat.py         # LLM-Chatbot (Gemini + LangChain)
        │   └── ui.py
        └── launch/llm_agent.launch.py
```

---

## Abhängigkeiten

### ROS2-Pakete

- `nav2_bringup` – Navigation Stack
- `turtlebot3_gazebo` – TurtleBot3 Simulation
- `tf2_ros` – Koordinaten-Transformationen
- `nav2_msgs`, `sensor_msgs`, `nav_msgs`, `std_srvs`

### Python-Pakete

- `langchain`, `langchain-core`, `langchain-google-genai`
- `pydantic`
- `google-generativeai` (Gemini 2.5 Flash)

---

## Umgebungsvariablen

| Variable | Beschreibung | Pflicht |
|----------|-------------|---------|
| `GOOGLE_API_KEY` | API-Key für Google Gemini | Ja |
| `GEMINI_MODEL` | Modell-Name (Standard: `gemini-2.5-flash`) | Nein |
| `TURTLEBOT3_MODEL` | Robotermodell (`burger`, `waffle`) | Ja |
---

## Fehlerbehebung

**Gazebo startet nicht:**
```bash
# X11-Weiterleitung prüfen
echo $DISPLAY
xhost +local:docker
```

**Nav2 findet keine Karte:**
```bash
# Sicherstellen, dass full_bringup vor patrol gestartet wurde
ros2 topic list | grep map
```

**LLM-Service nicht verfügbar:**
```bash
# LLM Agent Node muss laufen
ros2 service list | grep llm_tools
```

**API-Key-Fehler:**
```bash
# Key in der Shell und im Container setzen
export GOOGLE_API_KEY="dein-key"
docker compose up  # Container neu starten
```
