# log_recorder
A ros2 wrapper node around the ros2 bag record API.

- Allows for dynamic changing of recorded topics with the use of a parameter file or cli. 

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



