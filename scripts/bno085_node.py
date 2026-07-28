#!/usr/bin/env python3
import math
import struct
import time
from collections import deque

import rospy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

FLAG = 0x7E
ESCAPE = 0x7D
PROTOCOL_SHTP = 0x01
INTER_BYTE_GAP = 0.00025  # spec requires >=100us between bytes host->BNO; 250us for margin
BOOT_DELAY = 0.35

CHANNEL_EXECUTABLE = 1
CHANNEL_CONTROL = 2
CHANNEL_INPUT_REPORTS = 3

EXECUTABLE_RESET = 0x01
SET_FEATURE_COMMAND = 0xFD
COMMAND_REQUEST = 0xF2
COMMAND_RESPONSE = 0xF1
ME_CALIBRATE = 0x07
SAVE_DCD = 0x06

ACCELEROMETER = 0x01
GYROSCOPE_CALIBRATED = 0x02
MAGNETIC_FIELD = 0x03
ROTATION_VECTOR = 0x05
GAME_ROTATION_VECTOR = 0x08 
GEOMAGNETIC_ROTATION_VECTOR = 0x09
TIMESTAMP_REBASE = 0xFA
BASE_TIMESTAMP = 0xFB

# report id -> total length in bytes (including the id), used to walk a batched cargo
REPORT_LENGTHS = {0xFB: 5, 0xFA: 5, 0x01: 10, 0x02: 10, 0x03: 10, 0x04: 10, 0x05: 14, 0x06: 10, 0x08: 12, 0x09: 14}

ORIENTATION_REPORTS = {'rotation_vector': ROTATION_VECTOR, 'game_rotation_vector': GAME_ROTATION_VECTOR, 'geomagnetic_rotation_vector': GEOMAGNETIC_ROTATION_VECTOR}
REPORTS_WITH_ACCURACY = {ROTATION_VECTOR, GEOMAGNETIC_ROTATION_VECTOR}

TIMEBASE_TICK = 0.0001  # SH-2 timestamps and delays are in units of 100us

Q8 = 1.0 / 256.0
Q9 = 1.0 / 512.0
Q4 = 1.0 / 16.0
Q12 = 1.0 / 4096.0
Q14 = 1.0 / 16384.0

def quat_multiply(a, b):
	ax, ay, az, aw = a
	bx, by, bz, bw = b
	return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)

def quat_conjugate(q):
	return (-q[0], -q[1], -q[2], q[3])

def quat_from_rpy(roll, pitch, yaw):
	cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
	cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
	cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
	return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)

def rotate_vector(q, v):
	qx, qy, qz, qw = q
	vx, vy, vz = v
	tx = 2.0 * (qy * vz - qz * vy)
	ty = 2.0 * (qz * vx - qx * vz)
	tz = 2.0 * (qx * vy - qy * vx)
	return (vx + qw * tx + qy * tz - qz * ty, vy + qw * ty + qz * tx - qx * tz, vz + qw * tz + qx * ty - qy * tx)

class FrameReader:
	def __init__(self, port):
		self.port = port
		self.cur = bytearray()
		self.in_frame = False
		self.escaped = False
		self.frames = deque()

	def poll(self):
		data = self.port.read(self.port.in_waiting or 1)
		arrival = time.monotonic()
		for c in data:
			if c == FLAG:
				if self.cur:
					self.frames.append((arrival, bytes(self.cur)))
					self.cur = bytearray()
				self.in_frame = True
				self.escaped = False
			elif not self.in_frame:
				continue
			elif self.escaped:
				self.cur.append(c ^ 0x20)
				self.escaped = False
			elif c == ESCAPE:
				self.escaped = True
			else:
				self.cur.append(c)

	def next_frame(self, deadline):
		while not self.frames:
			if time.monotonic() > deadline:
				return None
			self.poll()
		return self.frames.popleft()

def make_shtp(channel, seq, cargo):
	length = len(cargo) + 4
	return bytes([length & 0xFF, (length >> 8) & 0xFF, channel, seq]) + cargo

def parse_shtp(packet):
	if len(packet) < 4:
		return None
	length = (packet[0] | (packet[1] << 8)) & 0x7FFF
	return packet[2], packet[3], packet[4:length]

def frame_uart(packet):
	out = bytearray([FLAG, PROTOCOL_SHTP])
	for c in packet:
		if c == FLAG or c == ESCAPE:
			out.append(ESCAPE)
			out.append(c ^ 0x20)
		else:
			out.append(c)
	out.append(FLAG)
	return bytes(out)

def busy_wait(seconds):
	end = time.perf_counter() + seconds
	while time.perf_counter() < end:
		pass

def set_feature(report_id, interval_us):
	cargo = bytearray(17)
	cargo[0] = SET_FEATURE_COMMAND
	cargo[1] = report_id
	struct.pack_into('<I', cargo, 5, interval_us)
	return bytes(cargo)

def command_request(command, params=()):
	cargo = bytearray(12)
	cargo[0] = COMMAND_REQUEST
	cargo[2] = command
	for i, p in enumerate(params):
		cargo[3 + i] = p
	return bytes(cargo)

def me_calibrate(accel, gyro, mag):
	return command_request(ME_CALIBRATE, (int(accel), int(gyro), int(mag)))

class SensorClock:
	"""Maps the BNO timebase onto host time. host = sensor + offset + queueing, and queueing is
	always positive, so the smallest offset in a recent window is the least-delayed sample and the
	best estimate. The window has to slide because the two crystals drift against each other."""

	def __init__(self, window, tolerance):
		self.window = window
		self.tolerance = tolerance
		self.samples = deque()
		self.trusted = True

	def update(self, host_time, sensor_time):
		self.samples.append((host_time, host_time - sensor_time))
		while self.samples and host_time - self.samples[0][0] > self.window:
			self.samples.popleft()
		offsets = [o for _, o in self.samples]
		lowest = min(offsets)
		self.trusted = (max(offsets) - lowest) <= self.tolerance
		if not self.trusted:
			return host_time
		return sensor_time + lowest

	def reset(self):
		self.samples.clear()
		self.trusted = True

class Bno085Node:
	def __init__(self):
		self.port_name = rospy.get_param('~port', '/dev/ttyUSB0')
		self.baud = rospy.get_param('~baud', 3000000)
		self.frame_id = rospy.get_param('~frame_id', 'imu_link')
		rate_hz = float(rospy.get_param('~rate', 100.0))
		self.interval_us = int(1000000 / rate_hz)

		report_name = rospy.get_param('~orientation_report', 'rotation_vector')
		if report_name not in ORIENTATION_REPORTS:
			rospy.logwarn('unknown orientation_report %r, falling back to rotation_vector', report_name)
			report_name = 'rotation_vector'
		self.orientation_report = ORIENTATION_REPORTS[report_name]
		self.report_name = report_name

		self.cal_accel = rospy.get_param('~calibrate_accel', True)
		self.cal_gyro = rospy.get_param('~calibrate_gyro', True)
		self.cal_mag = rospy.get_param('~calibrate_mag', False)
		self.force_save = rospy.get_param('~force_save', False)

		self.orientation_stddev = float(rospy.get_param('~orientation_stddev', 0.05))
		self.orientation_stddev_floor = float(rospy.get_param('~orientation_stddev_floor', 0.01))
		self.angular_velocity_stddev = float(rospy.get_param('~angular_velocity_stddev', 0.005))
		self.linear_acceleration_stddev = float(rospy.get_param('~linear_acceleration_stddev', 0.05))

		self.world_rotation = quat_from_rpy(*[float(v) for v in rospy.get_param('~world_rotation', [0.0, 0.0, 0.0])])
		self.mount_rotation = quat_from_rpy(*[float(v) for v in rospy.get_param('~mount_rotation', [0.0, 0.0, 0.0])])
		self.mount_inverse = quat_conjugate(self.mount_rotation)

		self.use_sensor_time = rospy.get_param('~use_sensor_time', True)
		self.clock = SensorClock(float(rospy.get_param('~clock_window', 10.0)), float(rospy.get_param('~clock_tolerance', 0.05)))
		self.stall_timeout = float(rospy.get_param('~stall_timeout', 0.5))
		self.diagnostic_period = 1.0 / float(rospy.get_param('~diagnostic_rate', 1.0))

		self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=50)
		self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
		self.calibrate_sub = rospy.Subscriber('~calibrate', Bool, self.calibrate_callback, queue_size=5)

		self.port = None
		self.reader = None
		self.sequence = {}
		self.timebase = 0
		self.command_responses = deque(maxlen=16)

		self.calibration_requested = False
		self.calibrating = False
		self.last_save_result = 'never'

		self.latest_gyro = None
		self.latest_accel = None
		self.rv_status = 0
		self.rv_accuracy = float('nan')
		self.mag_status = 0
		self.mag_field = float('nan')
		self.report_count = 0
		self.measured_rate = 0.0
		self.last_report_time = 0.0
		self.rate_window_start = 0.0
		self.stall_resends = 0
		self.next_diagnostic = 0.0

	def calibrate_callback(self, msg):
		# only records intent; every serial write happens on the main thread because send_packet
		# busy-waits per byte and two writers would interleave garbage on the wire
		self.calibration_requested = msg.data

	def send(self, channel, cargo):
		seq = self.sequence.get(channel, 0)
		self.sequence[channel] = (seq + 1) & 0xFF
		for b in frame_uart(make_shtp(channel, seq, cargo)):
			self.port.write(bytes([b]))
			self.port.flush()
			busy_wait(INTER_BYTE_GAP)

	def soft_reset(self):
		self.send(CHANNEL_EXECUTABLE, bytes([EXECUTABLE_RESET]))
		time.sleep(BOOT_DELAY)
		self.port.reset_input_buffer()
		self.timebase = 0
		self.clock.reset()
		self.command_responses.clear()

	def configure(self):
		if self.calibrating:
			self.send(CHANNEL_CONTROL, me_calibrate(True, True, True))
		else:
			self.send(CHANNEL_CONTROL, me_calibrate(self.cal_accel, self.cal_gyro, self.cal_mag))
		self.send(CHANNEL_CONTROL, set_feature(self.orientation_report, self.interval_us))
		self.send(CHANNEL_CONTROL, set_feature(ACCELEROMETER, self.interval_us))
		self.send(CHANNEL_CONTROL, set_feature(GYROSCOPE_CALIBRATED, self.interval_us))
		if self.calibrating:
			self.send(CHANNEL_CONTROL, set_feature(MAGNETIC_FIELD, self.interval_us))

	def start_stream(self):
		attempt = 0
		while not rospy.is_shutdown():
			attempt += 1
			self.soft_reset()
			self.configure()
			deadline = time.monotonic() + 1.0
			while time.monotonic() < deadline:
				frame = self.reader.next_frame(deadline)
				if frame is None:
					continue
				if self.handle_frame(frame) and self.last_report_time:
					rospy.loginfo('streaming %s at %d us (attempt %d), cal accel=%s gyro=%s mag=%s', self.report_name, self.interval_us, attempt, self.cal_accel, self.cal_gyro, self.cal_mag)
					self.stall_resends = 0
					return
			rospy.logwarn_throttle(5.0, 'no reports from the BNO after %d attempt(s); check the FT-TX -> BNO-RX wire and PS1=high/PS0=low' % attempt)

	def handle_frame(self, frame):
		arrival, data = frame
		if not data or data[0] != PROTOCOL_SHTP:
			return False
		parsed = parse_shtp(data[1:])
		if parsed is None:
			return False
		channel, _seq, cargo = parsed
		if channel == CHANNEL_CONTROL:
			if len(cargo) >= 6 and cargo[0] == COMMAND_RESPONSE:
				self.command_responses.append(bytes(cargo))
			return False
		if channel != CHANNEL_INPUT_REPORTS:
			return False
		return self.handle_input_reports(arrival, cargo)

	def handle_input_reports(self, arrival, cargo):
		got_orientation = False
		i = 0
		while i < len(cargo):
			rid = cargo[i]
			length = REPORT_LENGTHS.get(rid)
			if length is None or i + length > len(cargo):
				break
			report = cargo[i:i + length]
			i += length
			if rid == BASE_TIMESTAMP or rid == TIMESTAMP_REBASE:
				# deltas accumulate into a free-running sensor clock; only its rate matters, the
				# arbitrary origin is absorbed by the offset estimate in SensorClock
				self.timebase += struct.unpack_from('<i', report, 1)[0]
				continue
			status = report[2]
			delay = ((status >> 2) << 8) | report[3]
			sensor_time = (self.timebase + delay) * TIMEBASE_TICK
			if rid == ACCELEROMETER:
				self.latest_accel = tuple(v * Q8 for v in struct.unpack_from('<3h', report, 4))
			elif rid == GYROSCOPE_CALIBRATED:
				self.latest_gyro = tuple(v * Q9 for v in struct.unpack_from('<3h', report, 4))
			elif rid == MAGNETIC_FIELD:
				mx, my, mz = struct.unpack_from('<3h', report, 4)
				self.mag_status = status & 0x03
				self.mag_field = math.sqrt(mx * mx + my * my + mz * mz) * Q4
			elif rid == self.orientation_report:
				self.publish_imu(report, status & 0x03, arrival, sensor_time)
				got_orientation = True
		return got_orientation

	def publish_imu(self, report, accuracy_level, arrival, sensor_time):
		if self.orientation_report in REPORTS_WITH_ACCURACY:
			qi, qj, qk, qr, acc = struct.unpack_from('<5h', report, 4)
			self.rv_accuracy = acc * Q12
		else:
			qi, qj, qk, qr = struct.unpack_from('<4h', report, 4)
			self.rv_accuracy = float('nan')
		self.rv_status = accuracy_level

		q_sensor = (qi * Q14, qj * Q14, qk * Q14, qr * Q14)
		q = quat_multiply(quat_multiply(self.world_rotation, q_sensor), self.mount_rotation)

		if self.use_sensor_time:
			host_time = self.clock.update(arrival, sensor_time)
		else:
			host_time = arrival
			
		msg = Imu()
		msg.header.stamp = rospy.Time.now() - rospy.Duration(max(0.0, time.monotonic() - host_time))
		msg.header.frame_id = self.frame_id
		msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q

		accuracy = self.rv_accuracy if self.rv_accuracy == self.rv_accuracy else self.orientation_stddev
		variance = max(accuracy, self.orientation_stddev_floor) ** 2
		msg.orientation_covariance = [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]

		if self.latest_gyro is not None:
			gx, gy, gz = rotate_vector(self.mount_inverse, self.latest_gyro)
			msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
			var = self.angular_velocity_stddev ** 2
			msg.angular_velocity_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
		else:
			msg.angular_velocity_covariance = [-1.0] + [0.0] * 8

		if self.latest_accel is not None:
			ax, ay, az = rotate_vector(self.mount_inverse, self.latest_accel)
			msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax, ay, az
			var = self.linear_acceleration_stddev ** 2
			msg.linear_acceleration_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
		else:
			msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8

		self.imu_pub.publish(msg)
		now = time.monotonic()
		self.last_report_time = now
		self.report_count += 1
		if now - self.rate_window_start >= 1.0:
			self.measured_rate = self.report_count / (now - self.rate_window_start)
			self.report_count = 0
			self.rate_window_start = now

	def wait_for_command_response(self, command, timeout):
		deadline = time.monotonic() + timeout
		while time.monotonic() < deadline and not rospy.is_shutdown():
			for cargo in list(self.command_responses):
				if cargo[2] == command:
					self.command_responses.remove(cargo)
					return cargo[5] == 0  # byte 5 (R0) is command status, 0 = success
			frame = self.reader.next_frame(deadline)
			if frame is not None:
				self.handle_frame(frame)
		return False

	def enter_calibration(self):
		rospy.loginfo('calibration started: slow figure-8s through all axes, away from metal and electronics')
		self.calibrating = True
		self.mag_status = 0
		self.mag_field = float('nan')
		self.configure()

	def finish_calibration(self):
		converged = self.mag_status >= 2 and self.rv_status >= 2
		if converged or self.force_save:
			if not converged:
				rospy.logwarn('saving anyway (force_save): mag_cal=%d rv_status=%d', self.mag_status, self.rv_status)
			self.command_responses.clear()
			self.send(CHANNEL_CONTROL, command_request(SAVE_DCD))
			if self.wait_for_command_response(SAVE_DCD, 1.0):
				self.last_save_result = 'saved'
				rospy.loginfo('Save DCD confirmed (mag_cal=%d rv_status=%d rv_acc=%.3f rad), auto-restores on every boot', self.mag_status, self.rv_status, self.rv_accuracy)
			else:
				self.last_save_result = 'no response'
				rospy.logerr('no success response to Save DCD; calibration may not have persisted')
		else:
			self.last_save_result = 'skipped, not converged'
			rospy.logwarn('not saving: calibration did not converge (mag_cal=%d, rv_status=%d; need >=2 each). Move away from metal and retry, or set ~force_save.', self.mag_status, self.rv_status)
		self.calibrating = False
		self.start_stream()

	def publish_diagnostics(self):
		status = DiagnosticStatus()
		status.name = rospy.get_name().lstrip('/') + ': bno085'
		status.hardware_id = self.port_name
		stale = self.last_report_time and (time.monotonic() - self.last_report_time) > self.stall_timeout
		if stale:
			status.level = DiagnosticStatus.ERROR
			status.message = 'no reports'
		elif self.calibrating:
			status.level = DiagnosticStatus.WARN
			status.message = 'calibrating (mag_cal=%d, rv_status=%d)' % (self.mag_status, self.rv_status)
		elif self.rv_status >= 2:
			status.level = DiagnosticStatus.OK
			status.message = 'streaming'
		else:
			status.level = DiagnosticStatus.WARN
			status.message = 'orientation accuracy low (rv_status=%d)' % self.rv_status
		status.values = [
			KeyValue('mode', 'calibrating' if self.calibrating else 'running'),
			KeyValue('orientation_report', self.report_name),
			KeyValue('rv_status', str(self.rv_status)),
			KeyValue('rv_accuracy_rad', '%.4f' % self.rv_accuracy),
			KeyValue('mag_cal_status', str(self.mag_status)),
			KeyValue('mag_field_ut', '%.1f' % self.mag_field),
			KeyValue('measured_rate_hz', '%.1f' % self.measured_rate),
			KeyValue('sensor_clock', 'in use' if (self.use_sensor_time and self.clock.trusted) else 'host time'),
			KeyValue('last_save', self.last_save_result),
		]
		array = DiagnosticArray()
		array.header.stamp = rospy.Time.now()
		array.status = [status]
		self.diag_pub.publish(array)

	def spin(self):
		while not rospy.is_shutdown():
			frame = self.reader.next_frame(time.monotonic() + 0.02)
			if frame is not None:
				self.handle_frame(frame)
			now = time.monotonic()

			if self.calibration_requested != self.calibrating:
				if self.calibration_requested:
					self.enter_calibration()
				else:
					self.finish_calibration()
				continue

			if self.last_report_time and now - self.last_report_time > self.stall_timeout:
				self.stall_resends += 1
				if self.stall_resends > 3:
					rospy.logwarn('stream dead after %d resends, resetting the sensor', self.stall_resends - 1)
					self.start_stream()
				else:
					rospy.logwarn('stream stalled, resending set-feature')
					self.send(CHANNEL_CONTROL, set_feature(self.orientation_report, self.interval_us))
				self.last_report_time = time.monotonic()
			elif self.last_report_time and self.stall_resends:
				self.stall_resends = 0

			if now >= self.next_diagnostic:
				self.publish_diagnostics()
				self.next_diagnostic = now + self.diagnostic_period

	def run(self):
		while not rospy.is_shutdown():
			try:
				with serial.Serial(self.port_name, self.baud, timeout=0.02) as port:
					rospy.loginfo('BNO085 UART-SHTP on %s @ %d baud', self.port_name, self.baud)
					self.port = port
					self.reader = FrameReader(port)
					self.last_report_time = 0.0
					self.start_stream()
					self.spin()
			except (serial.SerialException, OSError) as e:
				rospy.logwarn_throttle(5.0, 'serial trouble on %s (%s), reopening' % (self.port_name, e))
				self.port = None
				self.reader = None
				rospy.sleep(1.0)

def main():
	rospy.init_node('bno085')
	Bno085Node().run()

if __name__ == '__main__':
	main()
