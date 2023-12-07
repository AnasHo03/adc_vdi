# Message for detected lane lines (currently dashed_line_detected is a placeholder)

# Bool for detected lane
bool right_lane_detected
bool left_lane_detected
bool dashed_line_detected

# Mean curvature from left and right lane lines (place holder)
#float32 curvature_radius

# Offset to center point
float32 center_offset

# Heading angle
float32 heading_angle

# Array of detected lane line points
float32[] right_lane_x_points
float32[] right_lane_y_points
float32[] left_lane_x_points
float32[] left_lane_y_points