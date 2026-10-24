# /compass/events — события для ленты в Compass (reliable, volatile, depth 50).
string id
builtin_interfaces/Time stamp
string level                     # info | warning | error
string code                      # MODE_CHANGED, CONTROL_TAKEN, …
string message
diagnostic_msgs/KeyValue[] details
