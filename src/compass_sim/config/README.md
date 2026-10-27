# Конфиги робота для Compass — образец (TurtleBot4 в симуляции)

Compass запускается так:

```bash
ros2 launch compass_bringup compass.launch.py robot_config:=<этот каталог> [cmd_vel_topic:=…] [data_dir:=…]
```

Для своего робота скопируйте каталог (например, в пакет своего робота) и поправьте файлы.
Драйверы робота — база (ros2_control), лидар, robot_state_publisher — запускаются отдельно и должны давать:
TF `odom → base_link` и `base_link → <фрейм лидара>`, топики лидара и одометрии, приём скорости
в `cmd_vel_topic` (`Twist` или `TwistStamped` — флаг `cmd_vel_stamped`).

| Файл | Что | Что править под своего робота |
|---|---|---|
| `compass_core.yaml` | фреймы, топики, тип скорости, начальные имя и лимиты скорости | `base_frame`, `scan_topic`, `odom_topic`, `battery_topic`; `cmd_vel_stamped` (база ждёт `TwistStamped`), `holonomic` (меканум, омни); `robot_radius` (значок на карте); имя, серийный номер, модель |
| `nav2.yaml` | Nav2: AMCL, костмапы, планировщик, контроллер | футпринт / радиус, скорости и ускорения, лидар, `base_frame_id`. **Обязательно**: `collision_monitor.cmd_vel_out_topic: cmd_vel_auto` — вход twist_mux от Nav2; `enable_stamped_cmd_vel` — как `cmd_vel_stamped` |
| `slam_toolbox.yaml` | картографирование (TEACH_IN) | `base_frame`, `scan_topic`, `max_laser_range` |
| `compass_web.yaml` | веб-сервер: порт, путь к собранному UI | `port` (80 на роботе), `ui_dir` |
| `vda5050.yaml` | агент VDA 5050 (`vda5050_ros2`): брокер fms, `manufacturer` / `serial_number`, factsheet | брокер, идентификаторы, габариты и скорости для factsheet; `simulated_action: false` на роботе |

Нельзя менять (их ждёт Compass): вход twist_mux `cmd_vel_auto` от Nav2, `map_frame: map`,
`odom_frame: odom` — если у робота другие имена, поправьте и `nav2.yaml`, и `compass_core.yaml`.

## Симуляция

`ros2 launch compass_sim sim.launch.py [headless:=True] [vda5050:=true]` — TurtleBot4 в Gazebo + Compass
(+ агент VDA 5050). В Gazebo TB4 нет BMS: заряд даёт `fake_battery.py` (80 %, тратится в движении).

**Если узлы перестают видеть друг друга** (AMCL не локализуется после смены карты, `/clock` без
публикатора): DDS ищет узлы по всем сетевым интерфейсам машины (Wi-Fi, docker, VPN). Запускайте
симуляцию и все `ros2 …` в терминалах с

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

После аварийных остановок — `fastdds shm clean`.
