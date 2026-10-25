# /compass/teleop — джойстик из Compass, 10 Гц, пока джойстик отклонён.
string lease_id
float32 linear                   # м/с, compass_core обрежет по лимитам
float32 angular                  # рад/с
float32 lateral                  # м/с вбок, влево — плюс; только у holonomic-робота (меканум, омни)
