# /compass/nav_status — текущая или последняя цель (reliable, transient_local, 1).
string goal_id                   # id цели, выдаёт compass_core; пусто — цели не было
string source                    # compass — оператор Compass | fms — агент VDA 5050
string operator_name             # client_name владельца аренды, поставившего цель; для fms — пусто
string kind                      # goal | route
string mode                      # waypoints | line (для route)
Pose2D[] poses                   # для goal — одна
uint32 index                     # к какой точке едем
string state                     # active | succeeded | failed | canceled
float32 distance_remaining       # м до конца; -1 — неизвестно (UI не показывает)
float32 eta                      # с; -1 — неизвестно
uint16 recoveries
string error_code                # NO_PATH | STUCK | NAV_FAILED (по коду ошибки Nav2); пусто без ошибки
string error_message
