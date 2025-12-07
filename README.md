
# camera_subscriber

Paczka ROS2 do sterowania TurtleBotem na podstawie kliknięć w oknie kamery.  

---

## Węzły paczki

### 1. `camera_node.py`
- Wyświetla okno kamery (`512x700`)  
- Publikuje kliknięty punkt w oknie na temat `/point`  
- Kliknięcie lewym przyciskiem myszki rejestruje punkt  

### 2. `point_follower.py`
- Subskrybuje `/point` z węzła `camera_node`  
- Steruje robotem TurtleBot3 w Gazebo:
  - Punkt powyżej środka okna (`y < 256`) → robot jedzie do przodu (`linear.x = 0.2`)  
  - Punkt poniżej środka (`y ≥ 256`) → robot zatrzymuje się (`linear.x = 0`)  

---

## Instalacja paczki

1. Skopiuj repozytorium do workspace ROS2:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Ignacy12345/turtlebot_control.git

---

# Budowa workspace
cd ~/ros2_ws
colcon build
source install/setup.bash

# Uruchamianie turtlebota
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Uruchamianie kamery
ros2 run camera_subscriber camera_node

# Uruchamianie sterowania
ros2 run camera_subscriber point_follower

# koniec