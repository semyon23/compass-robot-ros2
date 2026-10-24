# /compass/pose — поза для карты в Compass, 10 Гц (best_effort, volatile).
builtin_interfaces/Time stamp
string frame                     # map | odom
float64 x
float64 y
float64 theta
float32 linear                   # фактическая скорость, м/с
float32 angular                  # рад/с
