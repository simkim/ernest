#!/usr/bin/python
import rospy
from serial import Serial
from sensor_msgs.msg import Imu, Range, Joy

def maprange(value, omin, omax, tmin, tmax):
	return (value-omin)/(omax-omin)*(tmax-tmin) + tmin

class ArduinoCommunicator:
	ser = None
	def __init__(self):
		self.ser = Serial('/dev/ernesthead', 115200)
	def loop(self):
		while True:
			line = self.ser.readline()
			yield line.strip()
		
def main():
	pub = rospy.Publisher('ernest_sensors/imu', Imu, queue_size=10)
	eyes = rospy.Publisher('ernest_sensors/eyes', Range, queue_size=10)
	nunchuck = rospy.Publisher('ernest_sensors/joy', Joy, queue_size=10)
	nunchuck_giro = rospy.Publisher('ernest_sensors/joy_giro', Imu, queue_size=10)
	comm = ArduinoCommunicator()
	rospy.init_node('ernest_sensors')
	for line in comm.loop():
		print line
		try:
			x, y, z, dis, joy_x, joy_y, giro_x, giro_y, giro_z, but_c, but_z = line.split(",")
			imu = Imu()
			imu.header.frame_id = "imu"
			imu.linear_acceleration.x = -float(x)
			imu.linear_acceleration.y = float(z)
			imu.linear_acceleration.z = float(y)
			r = Range()
			r.header.frame_id = "imu"
			r.radiation_type = 1
			r.field_of_view = 0.5
			r.min_range = 0.20
			r.max_range = 3
			r.range = float(dis)/100.0
			j = Joy()
			j.axes = [
				maprange(float(joy_x), 25, 226, -1, 1), 
				maprange(float(joy_y), 32, 223, -1, 1)
			]
			j.buttons = [int(but_c), int(but_z)]
			imu2 = Imu()
			imu2.header.frame_id = "imu"
			imu2.linear_acceleration.x = (float(giro_y) - 512) / 512.0
			imu2.linear_acceleration.y = -(float(giro_x) - 512) / 512.0
			imu2.linear_acceleration.z = -(float(giro_z) - 512) / 512.0
			pub.publish(imu)
			eyes.publish(r)
			nunchuck.publish(j)
			nunchuck_giro.publish(imu2)
		except ValueError:
			continue
	#rospy.spin()

if __name__ == "__main__":
	main()
