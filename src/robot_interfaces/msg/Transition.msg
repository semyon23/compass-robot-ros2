# Переход между режимами: compass_core переключает стеки, режим меняется в конце.
bool active                      # false — не переключаемся, остальные поля пустые
string from_mode
string to_mode
string step                      # starting_stack | stopping_stack
string stack                     # slam | nav
builtin_interfaces/Time started_at
