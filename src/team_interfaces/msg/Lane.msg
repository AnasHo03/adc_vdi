# Message for detected lane lines

# Bool for detected lane
bool right_lane_detected
bool left_lane_detected
bool dashed_line_detected

# mean curvature from left and right lane lines
float32 curvature_radius

# Offset to center point
float32 center_offset

# Array of detected lane line points
float32[] right_lane_x_points
float32[] right_lane_y_points
float32[] left_lane_x_points
float32[] right_lane_y_points