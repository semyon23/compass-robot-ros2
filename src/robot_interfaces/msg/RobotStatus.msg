# /compass/status — сводное состояние робота для Compass.
# Публикуется при изменении и раз в секунду (reliable, transient_local, depth 1).
builtin_interfaces/Time stamp
string operating_mode            # как /robot/operating_mode: STARTUP | AUTOMATIC | INTERVENED | MANUAL | SERVICE | TEACH_IN
string state                     # ready | switching | error
Transition transition
MapRef map                       # TEACH_IN — карта, которую строим (id пустой — новая); иначе — активная
Localization localization
Battery battery
ControlState control
bool estop                       # аппаратный или программный стоп активен
StatusError[] errors
MappingState mapping
StackState[] stacks              # пусто — ни SLAM, ни навигация не запущены
