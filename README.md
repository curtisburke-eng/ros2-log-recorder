# log_recorder
# Summary
This log recorder node will act as a wrapper around the ros2 bag record api.

# Description
This node stands up two services: 
- Service `/start_recording` which, when called:
	- Generates a filename for the log recording based on the date and time
	- Reads the list of topics from the ros parameters file
	- Starts the ros2 bag recorder with the topics and destination filename
- Service `/stop_recording` which, when called:
	- Terminates the ros2 bag record subprocess

This node also allows for the dynamic editing of topics to record both via the `param.yaml` configuration file or via `ros2 set param` console command. 

# Instructions
## How to launch the node 
From the top level log_recorder directory
```
ros2 launch log_recorder log_recorder.launch.py
```

## How to call the services

### Start Recording

```
ros2 service call /start_recording std_srvs/srv/Trigger
```

### Stop Recording

```
ros2 service call /stop_recording std_srvs/srv/Trigger
```

## How to Dynamically Update Parameters from the CLI
After the node is running, use this command:
```
ros2 param set /log_recorder topic_list ["<topic-string1>", "<topic-string2>", ... , "<topic-stringN>"]
```
Where `<topic-string>` includes the / of the topic

Example:
```
ros2 param set /log_recorder topic_list ["/tf", "/tf_static"]
```



