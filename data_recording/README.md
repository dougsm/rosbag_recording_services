# README
This is a simple ROS package that creates services to start and stop rosbag recordings via service calls, which is useful for performing repeated data collection.

## Config
The config for the recording is in config/config.yaml.
Configure the output directory for the rosbag files and topics to be recorded (regex syntax is ok).
Rosbag files are saved with the current date and time.

## Run
`roslaunch data_recording data_recording.launch`

The package creates three services, which can be called with a std_srvs.srv.TriggerRequest message:
/data\_recording/start\_recording
/data\_recording/stop\_recording
/data\_recording/toggle\_recording
