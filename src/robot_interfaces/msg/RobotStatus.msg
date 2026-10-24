# /compass/status — сводное состояние робота для Compass.
# Публикуется при изменении и раз в секунду (reliable, transient_local, depth 1).
builtin_interfaces/Time stamp
string operating_mode            # как /robot/operating_mode: STARTUP | AUTOMATIC | INTERVENED | MANUAL | SERVICE | TEACH_IN
string state                     # ready | switching | error
Battery battery
ControlState control
bool estop                       # аппаратный или программный стоп активен
StatusError[] errors
# Срез 2 добавит: transition, map, localization, stacks, mapping.
