#########################################################
## Kudos : https://github.com/rasheeddo/ddsm115_python ##
#########################################################


import serial
import serial.rs485 #komunikasi serial RS485
import struct #pengolahan data terstruktur
import crcmod.predefined #perhitungan CRC
import numpy as np
import time
import csv

## PRINT FUNCTION ##
def print_info(text):
	print("DDSM115_INFO | {}".format(text))

def print_warning(text):
	print("DDSM115_WARNING | {}".format(text))

## MOTOR CONTROL CLASS ##
class MotorControl:

	## INISIALISASI OBJEK MOTORCONTROL ##
	def __init__(self, device="COM3"):

		## OBJEK SERIAL COM (device, baudrate, timeout) ##
		# self.ser = serial.Serial(device, 115200)
		self.ser = serial.rs485.RS485(device, 115200, timeout=0) 
		self.ser.rs485_mode = serial.rs485.RS485Settings()

		## SET FUNGSI CRC DGN ALGORITMA CRC-8-MAXIM ##
		self.crc8 = crcmod.predefined.mkPredefinedCrcFun('crc-8-maxim')

		## SET STRING, 10 & 9 byte ##
		self.str_10bytes = ">BBBBBBBBBB"
		self.str_9bytes = ">BBBBBBBBB"

		## SIMPAN FEEDBACK RPM & ARUS SEBELUMNYA ##
		self.prev_fb_rpm = [0,0,0,0]
		self.prev_fb_cur = [0,0,0,0]

	## FUNGSI CLOSE SERIAL ##
	def close(self):
		self.ser.close()

	######################
	#### FUNGSI MATHS ####
	######################
 
	## FUNGSI KONVERSI DATA int -> array byte (len = 2)##
	def Int16ToBytesArray(self, data: int):
		byte1 = (data & 0xFF00) >> 8
		byte2 = (data & 0x00FF)
		return [byte1, byte2]

	## FUNGSI KONVERSI DATA array byte (len = 2) -> int 16-bit ##
	def TwoBytesTo16Int(self, high_byte: int, lo_byte: int):
		int16 = ((high_byte & 0xFF)) << 8 | (lo_byte & 0xFF)
		return np.int16(int16).item()

	## FUNGSI SCALE NILAI DARI RENTANG A KE RENTANG B ##
	def map(self, val, in_min, in_max, out_min, out_max):
		return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	## FUNGSI MENAMBAHKAN CRC KE DATA BYTE ##
	def crc_attach(self, data_bytes: bytes):
		crc_int = self.crc8(data_bytes)
		data_bytesarray = bytearray(data_bytes)
		data_bytesarray.append(crc_int)
		full_cmd = bytes(data_bytesarray)
		
		return full_cmd

	## FUNGSI KONVERSI RAW CURRENT KE AMPERE rentang -8 sampai 8 A ##
	def currentRawToCurrentAmp(self, cur_raw: int):
		return self.map(cur_raw, -32767, 32767, -8.0, 8.0)

	######################
	### SEND PERINTAH ###
	######################
 
	## FUNGSI SET ID MOTOR ##
	def set_id(self, _id: int):
		"""
	 	connect only 1 motor, and call this function to set the ID of that motor
		"""

		SET_ID = struct.pack(self.str_10bytes, 0xAA, 0x55, 0x53, _id, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE)
		for i in range(5):
			self.ser.write(SET_ID)

	## FUNGSI KIRIM RPM ##
	def send_rpm(self, _id: int, rpm):

		rpm = int(rpm)
		rpm_ints = self.Int16ToBytesArray(rpm)
		cmd_bytes = struct.pack(self.str_9bytes, _id, 0x64, rpm_ints[0], rpm_ints[1], 0x00, 0x00, 0x00, 0x00, 0x00)
		cmd_bytes = self.crc_attach(cmd_bytes)

		while not self.ser.writable():
			print_warning("send_rpm not writable")
			pass
		self.ser.write(cmd_bytes)

		_,_,_ = self.read_reply(_id)
		# res = self.ser.read_until(size=10)
		# print(cmd_bytes)
		# print_info(res)

	## FUNGSI KIRIM DEGREE/SUDUT ORIENTASI ##
	def send_degree(self, _id: int, deg):
		"""
		Args:
		- deg: in degrees, 0 to 360.
		Absolute angle position control.
		
		"""

		raw = int(self.map(deg, 0, 360, 0, 32767))

		deg_ints = self.Int16ToBytesArray(raw)
		cmd_bytes = struct.pack(self.str_9bytes, _id, 0x64, deg_ints[0], deg_ints[1], 0x00, 0x00, 0x00, 0x00, 0x00)
		cmd_bytes = self.crc_attach(cmd_bytes)

		self.ser.write(cmd_bytes)
		_,_,_ = self.read_reply(_id)
		# res = self.ser.read_until(size=10)
		# print(cmd_bytes)

	## FUNGSI SET BRAKE ##
	def set_brake(self, _id: int):

		cmd_bytes = struct.pack(self.str_9bytes, _id, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00)
		cmd_bytes = self.crc_attach(cmd_bytes)
		self.ser.write(cmd_bytes)
		res = self.ser.read_until(size=10)

	## FUNGSI SET MODE ##
	def set_drive_mode(self, _id: int, _mode: int):
		"""
		_mode: 0x01 current (torque), 0x02 velocity, 0x03 position
		"""

		if _mode == 1:
			print_info(f"Set {_id} as current (torque) mode")
		elif _mode == 2:
			print_info(f"Set {_id} as velocity mode")
		elif _mode == 3:
			print_info(f"Set {_id} as position mode")
		else:
			print_info(f"Error {_mode} is unknown")

		cmd_bytes = struct.pack(self.str_10bytes, _id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, _mode)
		self.ser.write(cmd_bytes)

	## FUNGSI GET MOTOR ID ##
	def get_motor_id(self):

		ID_QUE = struct.pack(self.str_10bytes, 0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE)
		self.ser.write(ID_QUE)
		data = self.ser.read_until(size=10)
		# print(data)
		print_info(f"ID: {data[0]}")
		print_info(f"Mode: {data[1]}")
		print_info(f"Error: {data[8]}")

	## FUNGSI GET MOTOR FEEDBACK ##
	def get_motor_feedback(self, _id: int):
		fb_req_cmd = struct.pack(self.str_9bytes, _id, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
		fb_req_cmd = self.crc_attach(fb_req_cmd)
		while not self.ser.writable():
			print_warning("get_motor_feedback not writable")
			pass
		self.ser.write(fb_req_cmd)

		fb_rpm, fb_cur, error = self.read_reply(_id)
		if error != 0:
			sensor_error = error & 0b00000001
			over_current_error = error & 0b00000010
			phase_over_error = error & 0b00000100
			stall_error = error & 0b00001000
			troubleshoot_error = error & 0b00001000
			print_warning(f"error {error}")
			print_warning(f"sens_err: {sensor_error} phase_err: {phase_over_error} stall_err: {stall_error} trbs_err: {troubleshoot_error}")

		# Menampilkan output ampere yang digunakan tiap motor
		print(f"Motor ID {_id} Feedback: RPM = {fb_rpm}, Current = {fb_cur:.2f} A, Error Code = {error}")

		return fb_rpm, fb_cur

	## FUNGSI BACA REPLY ##
	def read_reply(self, _id, timeout=0.5):
		"""
		Read a reply immediately after sending a write command.
		This function uses a ring buffer concept to collect the serial data until it forms a complete packet.
		A complete packet is verified using CRC check and the expected message structure based on motor response.

		Args:
		- _id (int): The ID of the motor from which the reply is expected.
		- timeout (float): The timeout in seconds to wait for a complete reply before using the last known good values.

		Returns:
		- tuple: Contains feedback RPM, current in Amperes, and any error codes.
		"""
		got_reply = False
		ring_buffer = bytearray()
		start_time = time.time()

		while not got_reply and (time.time() - start_time) < timeout:
			try:
				res = self.ser.read()
				if res:
					# Start constructing the packet when the first byte (ID) matches the expected motor ID
					if len(ring_buffer) == 0 and res == _id.to_bytes(1, 'big'):
						ring_buffer.append(res[0])
					elif len(ring_buffer) > 0 and len(ring_buffer) < 10:
						ring_buffer.append(res[0])
					# When the buffer has collected 10 bytes, verify the packet
					if len(ring_buffer) == 10:
						crc_expected = ring_buffer[-1]
						data_to_check = bytes(ring_buffer[:-1])
						crc_calculated = self.crc8(data_to_check)
						if crc_expected == crc_calculated:
							got_reply = True
						else:
							print_warning("CRC error: Expected {}, got {}".format(crc_expected, crc_calculated))
							ring_buffer = bytearray()  # Reset the buffer on CRC error
				else:
					# If no data, briefly pause to yield time
					time.sleep(0.01)
			except serial.serialutil.SerialException as e:
				print_warning("Serial exception: {}".format(e))
				break

		if got_reply:
			ID = ring_buffer[0]
			mode = ring_buffer[1]
			cur_hi = ring_buffer[2]
			cur_lo = ring_buffer[3]
			rpm_hi = ring_buffer[4]
			rpm_lo = ring_buffer[5]
			error = ring_buffer[8]
			fb_cur = abs(self.currentRawToCurrentAmp(self.TwoBytesTo16Int(cur_hi, cur_lo)))
			fb_rpm = self.TwoBytesTo16Int(rpm_hi, rpm_lo)
			return fb_rpm, fb_cur, error
		
		else:
			# If no valid reply received within the timeout, use last known good values
			print_warning("Timeout: No valid response received. Returning last known values.")
			fb_rpm = self.prev_fb_rpm[_id-1]
			fb_cur = self.prev_fb_cur[_id-1]
			error = 0
			return fb_rpm, fb_cur, error

if __name__ == "__main__":
    # Inisialisasi objek kontrol motor
    motor_kiri = MotorControl(device="COM6")
    motor_kanan = MotorControl(device="COM5")
    
    # Mengatur mode pengendalian ke mode kecepatan (velocity mode)
    motor_kiri.set_drive_mode(_id=1, _mode=2)  # Pastikan _id sesuai dengan ID motor Anda
    motor_kanan.set_drive_mode(_id=1, _mode=2)
    try:
        while True:
			# Mengirimkan perintah RPM ke motor
            desired_rpm = 5 # Ganti dengan nilai RPM yang diinginkan
            motor_kiri.send_rpm(1, desired_rpm)
            motor_kanan.send_rpm(1, -(desired_rpm))
			
            #current_time=time.time()-start_time
			# Mendapatkan feedback arus setelah pengaturan RPM
            rpm_kiri, arus_kiri = motor_kiri.get_motor_feedback(1)
            rpm_kanan, arus_kanan = motor_kanan.get_motor_feedback(1)
			# Menampilkan hasil
            print(f"Motor Kiri: RPM = {rpm_kiri}, Arus = {arus_kiri} A")
            print(f"Motor Kanan: RPM = {-(rpm_kanan)}, Arus = {arus_kanan} A")


    except KeyboardInterrupt:
        motor_kiri.send_rpm(1, 0)
        motor_kanan.send_rpm(1, 0)
        print("Stop")
