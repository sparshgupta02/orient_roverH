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

## ik_callback
this function sets the ik_bool variables to true to falls based on which roll_controller will be called 
``` python
self.ik_bool = msg.data
```
## enc_callback
this function is used to set the variable enc_angle to the roll data recieved from the encoder 
``` python
self.enc_angle = msg.data[self.roll_constant]
```
## drawAxis
this function takes center(cntr) of the object and end point of one the edges(p1 and p2) in the pca direction to draw axes on the image 
in the code this function is used twice to get perpendiculars axes

``` python

def drawAxis(self, img, p_, q_, colour, scale):		#to draw a lines along the desired object
		p = list(p_)
		q = list(q_)
		angle = math.atan2(p[1] - q[1], p[0] - q[0])
		hypotenuse = math.sqrt((p[1] - q[1]) ** 2 + (p[0] - q[0]) ** 2)
		q[0] = p[0] - scale * hypotenuse * math.cos(angle)
		q[1] = p[1] - scale * hypotenuse * math.sin(angle)
		cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
		p[0] = q[0] + 9 * math.cos(angle + pi / 4)
		p[1] = q[1] + 9 * math.sin(angle + pi / 4)
		cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
		p[0] = q[0] + 9 * math.cos(angle - pi / 4)
		p[1] = q[1] + 9 * math.sin(angle - pi / 4)
		cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv.LINE_AA)
```
## getOrientation 
This function gets the orientation of the object using PCA and returns angle between the eigen vectors of the covariance matrix 
## roll_controller

``` python
if not math.isnan(self.angle):
			msg=Int32MultiArray()
			msg.data=[0,0,0,0,0,0]
			msg.layout = MultiArrayLayout()
			msg.layout.data_offset = 0
			msg.layout.dim = [MultiArrayDimension()]
			msg.layout.dim[0].size = msg.layout.dim[0].stride = len(msg.data)
			msg.layout.dim[0].label = 'write'

			print("Angle:", self.angle)
			if self.enc_angle < self.angle - 2:   # defining this as anticlockwise rotation
				msg.data[5] = -255
				print("roll angle:", self.enc_angle)
			elif self.enc_angle > self.angle + 2:
				msg.data[5] = 255
				print("roll angle:", self.enc_angle)
				self.rate.sleep()
			self.vel_pub.publish(msg)
			gripper_pub = 3 #set this to actual gripper publishing value while testing
			if abs(self.enc_angle - self.angle) < 2:
				msg.data[gripper_pub] = 255  #determine sign as per closing and opening
				self.vel_pub.publish(msg)
				rospy.sleep(5)
				self.om_pub.publish(True)
```

## Main function
