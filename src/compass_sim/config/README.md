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
| `compass_core.yaml` | фреймы, топики, тип скорости, начальные имя и лимиты скорости | `base_frame`, `scan_topic`, `odom_topic`, `battery_topic`; `cmd_vel_stamped` (база ждёт `TwistStamped`), `holonomic` (меканум, омни); имя, серийный номер, модель |
| `nav2.yaml` | Nav2: AMCL, костмапы, планировщик, контроллер | футпринт / радиус, скорости и ускорения, лидар, `base_frame_id`. **Обязательно**: `collision_monitor.cmd_vel_out_topic: cmd_vel_auto` — вход twist_mux от Nav2; `enable_stamped_cmd_vel` — как `cmd_vel_stamped` |
| `slam_toolbox.yaml` | картографирование (TEACH_IN) | `base_frame`, `scan_topic`, `max_laser_range` |
| `compass_web.yaml` | веб-сервер: порт, путь к собранному UI | `port` (80 на роботе), `ui_dir` |

Нельзя менять (их ждёт Compass): вход twist_mux `cmd_vel_auto` от Nav2, `map_frame: map`,
`odom_frame: odom` — если у робота другие имена, поправьте и `nav2.yaml`, и `compass_core.yaml`.
