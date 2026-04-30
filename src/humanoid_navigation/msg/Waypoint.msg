# 点位消息定义
string id
string name
string type  # navigation_target, exhibition_point, obstacle_point, etc.
float64[] position  # [x, y, z]
float64[] orientation  # [x, y, z, w]
string frame_id
string properties_json  # JSON格式的附加属性