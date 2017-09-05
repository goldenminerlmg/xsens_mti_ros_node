#!/usr/bin/env python
import roslib; roslib.load_manifest('xsens_driver')
import rospy
import select
import mtdevice
import math
import pdb

from std_msgs.msg import Header, Float32, Float64
from sensor_msgs.msg import Imu, Temperature, FluidPressure, NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped, Vector3Stamped, QuaternionStamped
from gps_common.msg import GPSFix, GPSStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from xsens_msgs.msg import sensorSample, baroSample, gnssSample
from xsens_msgs.msg import positionEstimate, velocityEstimate, orientationEstimate

# transform Euler angles or matrix into quaternions
from math import pi, radians
from tf.transformations import quaternion_from_matrix, quaternion_from_euler, identity_matrix

import numpy

def get_param(name, default):
	try:
		v = rospy.get_param(name)
		rospy.loginfo("Found parameter: %s, value: %s"%(name, str(v)))
	except KeyError:
		v = default
		rospy.logwarn("Cannot find value for parameter: %s, assigning "
				"default: %s"%(name, str(v)))
	return v

class XSensDriver(object):

	ENU = numpy.identity(3)
	NED = numpy.array([[0, 1, 0], [ 1, 0, 0], [0, 0, -1]])
	NWU = numpy.array([[0, 1, 0], [-1, 0, 0], [0, 0,  1]])

	def __init__(self):

		device = get_param('~device', 'auto')
		baudrate = get_param('~baudrate', 0)
		if device=='auto':
			devs = mtdevice.find_devices()
			if devs:
				device, baudrate = devs[0]
				rospy.loginfo("Detected MT device on port %s @ %d bps"%(device,baudrate))
			else:
				rospy.logerr("Fatal: could not find proper MT device.")
				rospy.signal_shutdown("Could not find proper MT device.")
				return
		if not baudrate:
			baudrate = mtdevice.find_baudrate(device)
		if not baudrate:
			rospy.logerr("Fatal: could not find proper baudrate.")
			rospy.signal_shutdown("Could not find proper baudrate.")
			return

		# Create our device
		rospy.loginfo("MT node interface: %s at %d bd."%(device, baudrate))
		self.mt = mtdevice.MTDevice(device, baudrate)

		# Configure the device (1=all, 2=rate, 3=filter)
		#self.mt.configureMti(200,1)

		# Get config values from ROS
		self.frame_id = get_param('~frame_id', 'mti/data')
		self.use_rostime = get_param('~use_rostime', True)
		self.frame_local = get_param('~frame_local', 'ENU')
		self.frame_local_imu = get_param('~frame_local_imu', 'ENU')

		if   self.frame_local == 'ENU':
			R = XSensDriver.ENU
		elif self.frame_local == 'NED':
			R = XSensDriver.NED
		elif self.frame_local == 'NWU':
			R = XSensDriver.NWU

		if   self.frame_local_imu == 'ENU':
			R_IMU = XSensDriver.ENU
		elif self.frame_local_imu == 'NED':
			R_IMU = XSensDriver.NED
		elif self.frame_local_imu == 'NWU':
			R_IMU = XSensDriver.NWU

		self.R = R.dot(R_IMU.transpose())

		# self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
		# self.diag_msg = DiagnosticArray()
		# self.stest_stat = DiagnosticStatus(name='mtnode: Self Test', level=1, message='No status information')
		# self.xkf_stat = DiagnosticStatus(name='mtnode: XKF Valid', level=1, message='No status information')
		# self.gps_stat = DiagnosticStatus(name='mtnode: GPS Fix', level=1, message='No status information')
		# self.diag_msg.status = [self.stest_stat, self.xkf_stat, self.gps_stat]

		# custom message types
		self.ss_pub = rospy.Publisher('custom/sample', sensorSample, queue_size=10) # sensorSample
		self.gnssPvt_pub = rospy.Publisher('custom/gnssPvt', gnssSample, queue_size=10) # GNSS PVT
		self.height_pub = rospy.Publisher('custom/height', baroSample, queue_size=10) # baro (height meters)
		# normal message types
		self.imuraw_pub = rospy.Publisher('sensor/imu_raw', Imu, queue_size=10) #IMU message
		self.imuins_pub = rospy.Publisher('sensor/imu_ins', Imu, queue_size=10) #IMU message
		self.mag_pub = rospy.Publisher('sensor/magnetic', Vector3Stamped, queue_size=10) # magnetic
		self.baro_pub = rospy.Publisher('sensor/pressure', FluidPressure, queue_size=10) # baro (fluid pressure)
		self.gps1_pub = rospy.Publisher('sensor/fix_navsat', NavSatFix, queue_size=10) # GNSS PVT
		#self.gps2_pub = rospy.Publisher('mti/sensor/fix_gpscommon', NavSatFix, queue_size=10) # GNSS PVT
		#self.gnssSatinfo_pub = rospy.Publisher('mti/sensor/fix_status', GPSStatus, queue_size=10) # GNSS SATINFO
		self.temp_pub = rospy.Publisher('sensor/temperature', Temperature, queue_size=10)	# decide type

		# xsens filters
		self.ori_pub = rospy.Publisher('filter/orientation', orientationEstimate, queue_size=10) # XKF/XEE orientation
		self.vel_pub = rospy.Publisher('filter/velocity', velocityEstimate, queue_size=10) # XKF/XEE velocity
		self.pos_pub = rospy.Publisher('filter/position', positionEstimate, queue_size=10) # XKF/XEE position

	def spin(self):
		try:
			while not rospy.is_shutdown():
				self.spin_once()
		# Ctrl-C signal interferes with select with the ROS signal handler
		# should be OSError in python 3.?
		except select.error:
			pass

	def spin_once(self):

		def baroPressureToHeight(value):
			c1 = 44330.0
			c2 = 9.869232667160128300024673081668e-6
			c3 = 0.1901975534856
			intermediate = 1-math.pow(c2*value, c3)
			height = c1*intermediate
			return height

		# get data
		data = self.mt.read_measurement()
		# common header
		h = Header()
		h.stamp = rospy.Time.now()
		h.frame_id = self.frame_id

		# get data (None if not present)
		temp_data = data.get('Temperature')	# float
		orient_data = data.get('Orientation Data')
		velocity_data = data.get('Velocity')
		position_data = data.get('Latlon')
		altitude_data = data.get('Altitude')
		acc_data = data.get('Acceleration')
		gyr_data = data.get('Angular Velocity')
		mag_data = data.get('Magnetic')
		pressure_data = data.get('Pressure')
		time_data = data.get('Timestamp')
		gnss_data = data.get('GNSS')

		# debug the data from the sensor
		# print(data)
		# print("\n")

		# create messages and default values
		"Temp message"
		temp_msg = Temperature()
		pub_temp = False
		"Imu message supported with Modes 1 & 2"
		imuraw_msg = Imu()
		pub_imuraw = False
		imuins_msg = Imu()
		pub_imuins = False
		"SensorSample message supported with Mode 2"
		ss_msg = sensorSample()
		pub_ss = False
		"Mag message supported with Modes 1 & 2"
		mag_msg = Vector3Stamped()
		pub_mag = False
		"Baro in meters"
		baro_msg = FluidPressure()
		height_msg = baroSample()
		pub_baro = False
		"GNSS message supported only with MTi-G-7xx devices"
		"Valid only for modes 1 and 2"
		gnssPvt_msg = gnssSample()
		gps1_msg = NavSatFix()
		gps2_msg = GPSFix()
		pub_gnssPvt = False
		gnssSatinfo_msg = GPSStatus()
		pub_gnssSatinfo = False
		# All filter related outputs
		"Supported in mode 3"
		ori_msg = orientationEstimate()
		pub_ori = False
		"Supported in mode 3 for MTi-G-7xx devices"
		vel_msg = velocityEstimate()
		pub_vel = False
		"Supported in mode 3 for MTi-G-7xx devices"
		pos_msg = positionEstimate()
		pub_pos = False

		# first getting the sampleTimeFine
		# note if we are not using ros time, the we should replace the header
		# with the time supplied by the GNSS unit
		if time_data and not self.use_rostime:
			time = time_data['SampleTimeFine']
			h.stamp.secs = 100e-6*time
			h.stamp.nsecs = 1e5*time - 1e9*math.floor(h.stamp.secs)

		# temp data
		if temp_data:
			temp_msg.temperature = temp_data['Temp']
			pub_temp = True

		# acceleration data
		if acc_data:
			if 'accX' in acc_data: # found acceleration
				pub_imuraw = True
				imuraw_msg.linear_acceleration.x = acc_data['accX']
				imuraw_msg.linear_acceleration.y = acc_data['accY']
				imuraw_msg.linear_acceleration.z = acc_data['accZ']
			if 'freeAccX' in acc_data: # found free acceleration
				pub_imuins = True
				imuins_msg.linear_acceleration.x = acc_data['freeAccX']
				imuins_msg.linear_acceleration.y = acc_data['freeAccY']
				imuins_msg.linear_acceleration.z = acc_data['freeAccZ']
			if 'Delta v.x' in acc_data: # found delta-v's
				pub_ss = True
				ss_msg.internal.imu.dv.x = acc_data['Delta v.x']
				ss_msg.internal.imu.dv.y = acc_data['Delta v.y']
				ss_msg.internal.imu.dv.z = acc_data['Delta v.z']
			else:
				raise MTException("Unsupported message in XDI_AccelerationGroup.")

		# gyro data
		if gyr_data:
			if 'gyrX' in gyr_data: # found rate of turn
				pub_imuraw = True
				imuraw_msg.angular_velocity.x = gyr_data['gyrX']
				imuraw_msg.angular_velocity.y = gyr_data['gyrY']
				imuraw_msg.angular_velocity.z = gyr_data['gyrZ']
				# note we do not force publishing the INS if we do not use the free acceleration
				imuins_msg.angular_velocity.x = gyr_data['gyrX']
				imuins_msg.angular_velocity.y = gyr_data['gyrY']
				imuins_msg.angular_velocity.z = gyr_data['gyrZ']
			if 'Delta q0' in gyr_data: # found delta-q's
				pub_ss = True
				ss_msg.internal.imu.dq.w = gyr_data['Delta q0']
				ss_msg.internal.imu.dq.x = gyr_data['Delta q1']
				ss_msg.internal.imu.dq.y = gyr_data['Delta q2']
				ss_msg.internal.imu.dq.z = gyr_data['Delta q3']
			else:
				raise MTException("Unsupported message in XDI_AngularVelocityGroup.")

		# magfield
		if mag_data:
			ss_msg.internal.mag.x = mag_msg.vector.x = mag_data['magX']
			ss_msg.internal.mag.y = mag_msg.vector.y = mag_data['magY']
			ss_msg.internal.mag.z = mag_msg.vector.z = mag_data['magZ']
			pub_mag = True

		if pressure_data:
			pub_baro = True
			baro_msg.fluid_pressure = pressure_data['Pressure']
			height = baroPressureToHeight(pressure_data['Pressure'])
			height_msg.height = ss_msg.internal.baro.height = height

		# gps fix message
		if gnss_data and 'lat' in gnss_data:
			pub_gnssPvt = True
			# A "3" means that the MTi-G is using the GPS data.
			# A "1" means that the MTi-G was using GPS data and is now coasting/dead-reckoning the
			# 	position based on the inertial sensors (the MTi-G is not using GPS data in this mode).
			# 	This is done for 45 seconds, before the MTi-G Mode drops to "0".
			# A "0" means that the MTi-G doesn't use GPS data and also that it
			# 	doesn't output position based on the inertial sensors.
			if gnss_data['fix'] < 2:
				gnssSatinfo_msg.status = NavSatStatus.STATUS_NO_FIX  # no fix
				gps1_msg.status.status = NavSatStatus.STATUS_NO_FIX  # no fix
				gps1_msg.status.service = 0
			else:
				gnssSatinfo_msg.status = NavSatStatus.STATUS_FIX  # unaugmented
				gps1_msg.status.status = NavSatStatus.STATUS_FIX  # unaugmented
				gps1_msg.status.service = NavSatStatus.SERVICE_GPS
			# lat lon alt
			gps1_msg.latitude = gnss_data['lat']
			gps1_msg.longitude = gnss_data['lon']
			gps1_msg.altitude = gnss_data['hEll']
			# covariances
			gps1_msg.position_covariance[0] = math.pow(gnss_data['horzAcc'],2)
			gps1_msg.position_covariance[4] = math.pow(gnss_data['horzAcc'],2)
			gps1_msg.position_covariance[8] = math.pow(gnss_data['vertAcc'],2)
			gps1_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
			# custom message
			gnssPvt_msg.itow = gnss_data['iTOW']
			gnssPvt_msg.fix = gnss_data['fix']
			gnssPvt_msg.latitude = gnss_data['lat']
			gnssPvt_msg.longitude = gnss_data['lon']
			gnssPvt_msg.hEll = gnss_data['hEll']
			gnssPvt_msg.hMsl = gnss_data['hMsl']
			gnssPvt_msg.vel.x = gnss_data['velE']
			gnssPvt_msg.vel.y = gnss_data['velN']
			gnssPvt_msg.vel.z = gnss_data['velD']
			gnssPvt_msg.hAcc = gnss_data['horzAcc']
			gnssPvt_msg.vAcc = gnss_data['vertAcc']
			gnssPvt_msg.sAcc = gnss_data['speedAcc']
			gnssPvt_msg.pDop = gnss_data['PDOP']
			gnssPvt_msg.hDop = gnss_data['HDOP']
			gnssPvt_msg.vDop = gnss_data['VDOP']
			gnssPvt_msg.numSat = gnss_data['nSat']
			gnssPvt_msg.heading = gnss_data['heading']
			gnssPvt_msg.headingAcc = gnss_data['headingAcc']


		if orient_data:
			if 'Q0' in orient_data:
				pub_imuraw = True
				imuraw_msg.orientation.w = orient_data['Q0']
				imuraw_msg.orientation.x = orient_data['Q1']
				imuraw_msg.orientation.y = orient_data['Q2']
				imuraw_msg.orientation.z = orient_data['Q3']
				pub_imuins = True
				imuins_msg.orientation.w = orient_data['Q0']
				imuins_msg.orientation.x = orient_data['Q1']
				imuins_msg.orientation.y = orient_data['Q2']
				imuins_msg.orientation.z = orient_data['Q3']
			elif 'Roll' in orient_data:
				pub_ori = True
				ori_msg.roll = orient_data['Roll']
				ori_msg.pitch = orient_data['Pitch']
				ori_msg.yaw = orient_data['Yaw']
			else:
				raise MTException('Unsupported message in XDI_OrientationGroup')

		if velocity_data:
			pub_vel = True
			vel_msg.velE = velocity_data['velX']
			vel_msg.velN = velocity_data['velY']
			vel_msg.velU = velocity_data['velZ']

		if position_data:
			pub_pos = True
			pos_msg.latitude = position_data['lat']
			pos_msg.longitude = position_data['lon']

		if altitude_data:
			pub_pos = True
			tempData = altitude_data['ellipsoid']
			pos_msg.hEll = tempData[0]

		# publish available information
		if pub_imuraw:
			imuraw_msg.header = h
			self.imuraw_pub.publish(imuraw_msg)
		if pub_imuins:
			imuins_msg.header = h
			self.imuins_pub.publish(imuins_msg)
		if pub_mag:
			mag_msg.header = h
			self.mag_pub.publish(mag_msg)
		if pub_temp:
			temp_msg.header = h
			self.temp_pub.publish(temp_msg)
		if pub_ss:
			ss_msg.header = h
			self.ss_pub.publish(ss_msg)
		if pub_baro:
			baro_msg.header = h
			height_msg.header = h
			self.baro_pub.publish(baro_msg)
			self.height_pub.publish(height_msg)
		if pub_gnssPvt:
			gnssPvt_msg.header = h
			gps1_msg.header = h
			self.gnssPvt_pub.publish(gnssPvt_msg)
			self.gps1_pub.publish(gps1_msg)
		#if pub_gnssSatinfo:
		#	gnssSatinfo_msg.header = h
		#	self.gnssSatinfo_pub.publish(gnssSatinfo_msg)
		if pub_ori:
			ori_msg.header = h
			self.ori_pub.publish(ori_msg)
		if pub_vel:
			vel_msg.header = h
			self.vel_pub.publish(vel_msg)
		if pub_pos:
			pos_msg.header = h
			self.pos_pub.publish(pos_msg)


def main():
	'''Create a ROS node and instantiate the class.'''
	rospy.init_node('xsens_driver')
	driver = XSensDriver()
	driver.spin()


if __name__== '__main__':
	main()


