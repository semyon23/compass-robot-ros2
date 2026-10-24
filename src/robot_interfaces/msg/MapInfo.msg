# Карта из библиотеки.
string id
string name
uint32 width                     # клетки
uint32 height
float32 resolution               # м/клетка
Pose2D origin                    # как origin в map.yaml
bool has_posegraph               # есть map.posegraph + map.data — можно продолжить картографирование
uint64 size_bytes
builtin_interfaces/Time created_at
builtin_interfaces/Time updated_at
