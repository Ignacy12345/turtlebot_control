
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
```


2. Budowa workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

3. Uruchamianie turtlebota
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

4. Uruchamianie kamery
```bash
ros2 run camera_subscriber camera_node
```

4. Uruchamianie sterowania
```bash
ros2 run camera_subscriber point_follower
```


## Odpalanie z launcha
```bash
ros2 launch camera_subscriber turtlebot.launch.py
```

---