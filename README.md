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


# Methods used

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
This function gets the orientation of the object using PCA and returns the direction pointed of the eigenvector with the highest variance basically giving the direction of the length of object , it even makes a circle of radius 3 and thickness 2 around the circle with the center of object being the center of the circle 

``` python
def getOrientation(self, pts, img):		#to get the orientation of a contour
		sz = len(pts)	#finding the number of points in the contour
		data_pts = np.empty((sz, 2), dtype=np.float64)	#initializing a numpy array to store the coordinates of contour points
		for i in range(data_pts.shape[0]):
			data_pts[i, 0] = pts[i, 0, 0]
			data_pts[i, 1] = pts[i, 0, 1]
		mean = np.empty((0))
		mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)	#eigenvector correspoding to the highest eigenvalue is the orientation in PCA calculation
		# PCA compute makes covariance matrix x* x(transpose) and gets its eigen values and eigen vecots 
		cntr = (int(mean[0, 0]), int(mean[0, 1]))               # finds center of the object using mean of the data points 
		cv.circle(img, cntr, 3, (255, 0, 255), 2)
		p1 = (cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0], cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])   # uses the largest eigenvalues corresponding eigenvector to get length
		p2 = (cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0], cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])  # uses the second largest eigenvalues corresponding eigenvector to get width	
		self.drawAxis(img, cntr, p1, (0, 255, 0), 1)          # draws the required axes in lenght
		self.drawAxis(img, cntr, p2, (255, 255, 0), 5)        # draws perpendicular axes in width 
		angle = math.atan2(eigenvectors[0, 1], eigenvectors[0, 0])
		return angle
```
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



# Changes 
- pranav_callback --> ik_callback
- pranav_bool --> ik_bool
- om_pub --> goal_reached_pub
- 
