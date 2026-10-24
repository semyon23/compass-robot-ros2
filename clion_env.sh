
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi
