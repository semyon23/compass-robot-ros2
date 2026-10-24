<p align="center">
  <img src="assets/banner.svg" alt="Compass — onboard robot software: service via Compass, fleet operation with fms over VDA 5050" width="100%">
</p>

# compass_ws — бортовое ПО робота

ROS 2 workspace со всем, что работает на ПК робота: сервисная часть для **Compass** (веб-консоль
обслуживания), агент **VDA 5050** для работы с fms, общие интерфейсы и сторонние пакеты в
зафиксированных версиях. Workspace переносится на робот целиком и собирается там одной командой.

**Стек:** Ubuntu 24.04 · ROS 2 Jazzy · Nav2 · slam_toolbox · C++.

## Как устроено

```
   Compass UI (браузер)                                    fms
          │ HTTP + WebSocket                                │ MQTT, VDA 5050 v3
┌─────────┼──────────────── ПК робота ──────────────────────┼─────────────────┐
│  nginx ─┴─ /rosbridge ─► rosbridge_suite                  │                 │
│                               │ ROS 2                     │                 │
│     compass_core ◄────────────┘           vda5050_adapter ┘                 │
│     режим работы, карты, конфиг,  ──/robot/operating_mode──►                 │
│     джойстик, СТОП                ◄──/robot/maps/activate───                 │
│           │                                     │          │                 │
│           ▼                                     ▼          ▼                 │
│     ros2_task_orchestrator ──► докинг, лифт,   Nav2: FollowPath по рёбрам    │
│     (сервисные цели, функции)  зарядка, цели                                 │
└──────────────────────────────────────────────────────────────────────────────┘
```

- **Compass** обслуживает робота: картографирование, ручное управление, настройки. Подключается
  к роботу по WebSocket через nginx.
- **fms** управляет флотом: шлёт order'а по VDA 5050, агент едет строго по рёбрам графа.
- **Кто командует роботом**, решает режим работы — `operatingMode` из VDA 5050:

| Режим | Кто командует |
|---|---|
| `AUTOMATIC` | fms через агента |
| `INTERVENED` | оператор посреди order'а (order сохраняется) |
| `MANUAL` | оператор, ручная езда |
| `SERVICE` | оператор, обслуживание |
| `TEACH_IN` | оператор, построение карты |

Подробно — в документации (ниже).

## Пакеты

| Пакет | Что | Состояние |
|---|---|---|
| `robot_interfaces` | общие msg/srv: режим, статус, аренда управления, джойстик, поза, скан | ✅ первый срез |
| `compass_core` | сервисная часть: режим работы, статус, аренда управления, джойстик, поза, скан (дальше — стеки, карты, конфиг) | ✅ срез 1 |
| `compass_bringup` | launch и конфиги: twist_mux, rosbridge, Nav2, задачи оркестратора — станет `robot_bringup` | заготовка |
| `vda5050_ros2` | агент VDA 5050 (ядро — из симулятора fms) | впереди |
| `third_party/ros2_task_orchestrator` | 1.0.0 — исполнитель задач | ✅ собирается |
| `third_party/rosbridge_suite` | 4.2.1 — WebSocket ↔ ROS для Compass | ✅ собирается |

Из `ros2_task_orchestrator` собираются только `task_orchestrator_msgs` и `task_orchestrator_core`.
Остальные его пакеты (симуляторы, ArduPilot, примеры) роботу не нужны и отключены файлами
`COLCON_IGNORE` — не удалены, чтобы пакет можно было обновить целиком.

## Сборка

```bash
source /opt/ros/jazzy/setup.bash
cd ~/compass_ws
colcon build --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`-DBUILD_TESTING=OFF` — тестам rosbridge нужен `ament_cmake_mypy`, роботу тесты не нужны.

Проверка интерфейсов:

```bash
ros2 interface package robot_interfaces
```

## CLion

1. Один раз собрать workspace colcon'ом (как выше, удобнее с `-DCMAKE_BUILD_TYPE=Debug` и
   `--symlink-install`).
2. **Settings → Build, Execution, Deployment → Toolchains → Add environment → From file** →
   `clion_env.sh` из этого workspace. Он подключает ROS и `install/`, иначе CLion не найдёт
   `rclcpp` и `robot_interfaces`.
3. **File → Open** → `src/compass_core/CMakeLists.txt` → **Open as Project**; в CMake-профиле —
   `Debug`, опции `-DBUILD_TESTING=OFF`.
4. Запуск и отладка ноды — кнопками CLion. Поменяли msg/srv — пересобрать `robot_interfaces`
   colcon'ом и **File → Reload CMake Project**.

## Перенос на робот

Копируем только исходники — `build/`, `install/`, `log/` собираются на месте:

```bash
rsync -a --delete --exclude build --exclude install --exclude log ~/compass_ws/ <робот>:~/compass_ws/
```

На роботе один раз — зависимости из `package.xml` всех пакетов:

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
cd ~/compass_ws
rosdep install --from-paths src --ignore-src -r -y
```

Дальше — сборка, как выше.

## Документация

| Что | Где |
|---|---|
| Архитектура бортового ПО: режимы, общие ресурсы, процессы, порядок работ | `~/fms_robot/ARCHITECTURE.md` |
| Агент VDA 5050: устройство, этапы, что ожидает fms | `~/fms_robot/README.md` |
| Контракт с Compass UI: топики, сервисы, msg/srv, белые списки rosbridge | `~/compass/compass-ui/docs/ROS_INTERFACES.md` |
| Палитра и стиль Compass (по ним сделан баннер) | `~/compass/compass-ui/docs/DESIGN.md` |
