# Окружение ROS для CLion: Settings → Build, Execution, Deployment → Toolchains →
# Add environment → From file → этот файл. Путь к workspace — от расположения файла,
# поэтому работает на любой машине (/home/semen, /home/ubuntu, …).
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
# install/ появляется после первой сборки colcon'ом.
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi
