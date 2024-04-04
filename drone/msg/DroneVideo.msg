# Video frame message definition. Represents a single video frame

string frame_id  # Unique identifier for the frame
sensor_msgs/Image image  # The actual video frame (using sensor_msgs/Image)
float64 timestamp  # Timestamp when the frame was captured
