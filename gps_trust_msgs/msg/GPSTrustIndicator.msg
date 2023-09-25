# GPSTrustIndicator.msg
std_msgs/Header header # Header timestamp should be the time of response
int8 trust_level       # GPS Trust indicator (0 - do not use, 10 - device is fully trusted)
string status          # Status message