#!/usr/bin/env python

import usb
import time

USB_VENDOR_ID = 0x16C0

def enumerate():
	"""This function enumerates attached little robots.
	"""
	devices = []
	buses = usb.busses()
	for bus in buses:
		for device in bus.devices:
			if device.idVendor == USB_VENDOR_ID:
				yield( bot( device ) )

class bot:
	HID_SET_REPORT = 0x09
	HID_GET_REPORT = 0x01
	HID_REPORT_TYPE_FEATURE = 3

	FEAT_POSITION = 0x01
	FEAT_CALIBRATE_LOW = 0x02
	FEAT_CALIBRATE_HIGH = 0x03
	FEAT_BUTTON_PRESSES = 0x04
	FEAT_NAME = 0x05

	def __init__(self, device ):
		if not device:
			raise Exception('Device not available')
		self.device = device
		self.handle = device.open()
		self.set_position( self.get_calibration_high() )
		time.sleep( .3 )

	def send_cmd(self, cmd, buffer, param=0):
		val = self.handle.controlMsg( requestType =
				usb.TYPE_CLASS | usb.RECIP_DEVICE | usb.ENDPOINT_OUT,
				request = cmd, value = param, buffer = buffer, timeout=500 )
		return val

	def recv_cmd(self, cmd, buffer, param=0):
		val = self.handle.controlMsg( requestType =
				usb.TYPE_CLASS | usb.RECIP_DEVICE | usb.ENDPOINT_IN,
				request = cmd, value = param, buffer = buffer, timeout=500 )
		return val

	def get_name( self ):
		return self.recv_cmd( self.HID_GET_REPORT, 32,
				param = ( self.HID_REPORT_TYPE_FEATURE<<8 ) | self.FEAT_NAME )

	def set_name( self, name ):
		self.send_cmd( self.HID_SET_REPORT, buffer = name,
				param = ( self.HID_REPORT_TYPE_FEATURE<<8 ) | self.FEAT_NAME)

	def set_position( self, angle ):
		if( angle >= self.get_calibration_low() and
				angle <= self.get_calibration_high() ):
			self.send_cmd( self.HID_SET_REPORT, buffer = [angle],
					param = ( self.HID_REPORT_TYPE_FEATURE<<8 ) | self.FEAT_POSITION)

	def set_calibration_low( self, angle):
		self.send_cmd( self.HID_SET_REPORT, buffer = [angle],
				param = ( self.HID_REPORT_TYPE_FEATURE<<8 ) | self.FEAT_CALIBRATE_LOW )

	def set_calibration_high( self, angle):
		self.send_cmd( self.HID_SET_REPORT, buffer = [angle],
				param = ( self.HID_REPORT_TYPE_FEATURE<<8 ) | self.FEAT_CALIBRATE_HIGH )
	
	def get_position( self ):
		return self.recv_cmd( self.HID_GET_REPORT, 1,
				param = ( self.HID_REPORT_TYPE_FEATURE<<8 ) | self.FEAT_POSITION)[0]

	def get_calibration_low( self ):
		return self.recv_cmd( self.HID_GET_REPORT, 1,
				param = ( self.HID_REPORT_TYPE_FEATURE<<8 ) | self.FEAT_CALIBRATE_LOW)[0]

	def get_calibration_high( self ):
		return self.recv_cmd( self.HID_GET_REPORT, 1,
				param = ( self.HID_REPORT_TYPE_FEATURE<<8 ) | self.FEAT_CALIBRATE_HIGH)[0]

	def get_button_presses( self ):
		return self.recv_cmd( self.HID_GET_REPORT, 1,
				param = ( self.HID_REPORT_TYPE_FEATURE<<8 ) | self.FEAT_BUTTON_PRESSES)[0]

	def wave( self ):
		self.set_position( self.get_calibration_low() )
		time.sleep( .3 )
		self.set_position( self.get_calibration_high() )
		time.sleep( .3 )

	def small_wave( self ):
		self.set_position( (self.get_calibration_high() + self.get_calibration_low()) / 2 )
		time.sleep( .3 )
		self.set_position( self.get_calibration_high() )
		time.sleep( .3 )


if __name__ == '__main__':
	client = enumerate().next()
	client.small_wave()
	time.sleep( 1 )
	client.wave()


