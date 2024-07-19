# orient_roverH
this is repository to explain orient_o file 

# Initialisation 
Initializes the script as a ROS node with name `go_to_goal`.

## Subscribers
- Subscribes to encoder readings on the topic `/enc_drive` (gets encoder data from motor)
- Subscribes to data on the topic `ik_over_ah` (this basically gets data to execute roll_controller or not)

## Publisher
- Publishes to topic `auto_arm_signals`
- Publishes to topic `om_bool`

## Variables used
- bashbsljbfhjb
- 


# Functions used

## pranav_callback
this function sets the ik_bool variables to true to falls based on which roll_controller will be called 
``` python
self.ik_bool = msg.data
```
## enc_callback
this function is used to set the 
