#!/usr/bin/env python3

import argparse
import binascii
import struct
import sys
import time

import serial


def crc(data):
	return binascii.crc32(data)


class MySerial:
	def __init__(self, file_dev):
		self.ser = serial.Serial(port=file_dev, baudrate=921600, bytesize=8, stopbits=serial.STOPBITS_ONE, timeout=500)

	def write_cmd(self, cmd):
		b = struct.pack('>B', cmd)
		self.ser.write(b)
		print(f"CMD: {hex(cmd)}")

	def read_resp(self, cmd):
		b = self.ser.read(1)
		if not b:
			raise RuntimeError("Timeout reading response")
		print(f"RESP_CMD: {b}")
		(r,) = struct.unpack('>B', b)
		if r != cmd + 0x80:
			raise RuntimeError("Unexpected response")
		print("RESP_OK")

	def read_n(self, n):
		b = self.ser.read(n)
		if not b:
			raise RuntimeError("Timeout reading response")
		if len(b) != n:
			raise RuntimeError(f"Incorrect data len: expected={n} got={len(b)}")
		return b

	def read_4(self):
		b = self.read_n(4)
		(r,) = struct.unpack('>I', b)
		return r

	def write(self, data):
		self.ser.write(data)

	def write_4(self, n):
		self.ser.write(struct.pack('>I', n))


cmd_ping = 0x10
cmd_checksum = 0x20
cmd_read_all = 0x30
cmd_erase_all = 0x40
cmd_write = 0x50
cmd_checksum_flash = 0x60
cmd_program_fpga_flash = 0x70


def run_ping(my_ser):
	my_ser.write_cmd(cmd_ping)
	my_ser.read_resp(cmd_ping)


def run_checksum(my_ser, file):
	with open(file, 'rb') as f:
		data = f.read()
	data_len = len(data)
	print(f"Len: {data_len}")

	my_ser.write_cmd(cmd_checksum)
	my_ser.write_4(data_len)
	time.sleep(0.1)
	my_ser.write(data)

	my_ser.read_resp(cmd_checksum)
	(crc_remote,) = struct.unpack('>I', my_ser.read_n(4))
	crc_local = crc(data)
	if crc_local == crc_remote:
		print(f"RESULT_OK: crc={hex(crc_local)}")
	else:
		print(f"RESULT_FAIL: expected={hex(crc_local)} got={hex(crc_remote)}")


def run_read_all(my_ser, file):
	my_ser.write_cmd(cmd_read_all)
	my_ser.read_resp(cmd_read_all)
	data_len = my_ser.read_4()
	print(f"RESP: data_len={data_len}")
	data = my_ser.read_n(data_len)
	crc_remote = my_ser.read_4()
	print(f"RESP: crc_remote={hex(crc_remote)}")
	crc_local = crc(data)
	print(f"crc_local={hex(crc_local)}")
	if crc_remote != crc_local:
		print("CRC mismatch")
		exit(1)
	print("CRC ok")
	with open(file, "wb") as f:
		f.write(data)


def run_erase_all(my_ser):
	my_ser.write_cmd(cmd_erase_all)
	my_ser.read_resp(cmd_erase_all)


def run_write(my_ser, file):
	with open(file, 'rb') as f:
		data = f.read()
	data_len = len(data)
	crc_local = crc(data)
	print(f"Len: {data_len}")
	print(f"CRC local: {hex(crc_local)}")

	# TODO: flow control, small chunks
	my_ser.write_cmd(cmd_write)
	my_ser.write_4(data_len)
	time.sleep(0.1)
	my_ser.write(data)
	my_ser.write_4(crc_local)

	my_ser.read_resp(cmd_checksum)
	my_ser.read_resp(cmd_write)
	print(f"RESULT_OK")


def run_checksum_flash(my_ser, file):
	with open(file, 'rb') as f:
		data = f.read()
	data_len = len(data)
	crc_local = crc(data)
	print(f"Len: {data_len}")
	print(f"CRC local: {hex(crc_local)}")

	my_ser.write_cmd(cmd_checksum_flash)
	my_ser.write_4(data_len)

	my_ser.read_resp(cmd_checksum_flash)
	crc_remote = my_ser.read_4()
	if crc_local == crc_remote:
		print("CRC_OK")
	else:
		print(f"CRC_ERROR: crc_remote={hex(crc_remote)}")


def run_program_fpga_flash(my_ser, file):
	with open(file, 'rb') as f:
		data = f.read()
	data_len = len(data)
	crc_local = crc(data)
	print(f"Len: {data_len}")
	print(f"CRC local: {hex(crc_local)}")

	my_ser.write_cmd(cmd_program_fpga_flash)
	my_ser.write_4(data_len)
	time.sleep(0.1)
	my_ser.write(data)
	my_ser.write_4(crc_local)

	my_ser.read_resp(cmd_checksum)
	my_ser.read_resp(cmd_erase_all)
	my_ser.read_resp(cmd_write)
	my_ser.read_resp(cmd_checksum_flash)
	my_ser.read_resp(cmd_program_fpga_flash)

	print("RESULT_OK")

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--device", metavar="UART_DEVICE")
parser.add_argument("command", metavar="CMD", type=str)
parser.add_argument("file", metavar="FILE", type=str)

if __name__ == "__main__":
	args = parser.parse_args()
	my_ser = MySerial(args.device)
	if args.command == 'ping':
		run_ping(my_ser)
	elif args.command == 'checksum':
		run_checksum(my_ser, args.file)
	elif args.command == 'read_all':
		run_read_all(my_ser, args.file)
	elif args.command == 'erase_all':
		run_erase_all(my_ser)
	elif args.command == 'write':
		run_write(my_ser, args.file)
	elif args.command == 'checksum_flash':
		run_checksum_flash(my_ser, args.file)
	elif args.command == 'program_fpga_flash':
		run_program_fpga_flash(my_ser, args.file)
	else:
		parser.print_help()

