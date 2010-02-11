#!/usr/bin/env python

import time
import threading, logging

import xmpp
import usbrobot

config = {
	'jid': 'local-bot@jabber.server',
	'password' : 'password goes here',
	'other' : 'remote-bot@jabber.server',
	'action': '/me waves'
}

class JabberCommunicator( threading.Thread ):
	def __init__( self ):
		threading.Thread.__init__( self )
		self._want_abort = False
		self.waves = 0
		self.robot = None
		self.jid = xmpp.JID( config['jid'] )

	def connect( self ):
		self.connection = xmpp.Client( self.jid.getDomain() )

		result = self.connection.connect()
		if result is None:
			logging.error( 'Could not connect to jabber server.' )

		result = self.connection.auth( self.jid.getNode(), config['password'] )
		if result is None:
			logging.error( 'Could not authenticate against jabber server.' )

		self.connection.sendInitPresence()
		self.connection.RegisterHandler( 'message',
				self.message_handler )

	def run( self ):
		self.connect()

		while( self._want_abort == False ):
			if not self.connection.isConnected():
				self.connection.reconnectAndReauth()
				self.connection.sendInitPresence()
				self.connection.RegisterHandler( 'message',
						self.message_handler )

			else:
				self.connection.Process( 0.1 )
				if( self.robot ):
					try:
						for i in range( self.robot.get_button_presses() ):
							self.send_wave()
					except:
						logging.info( 'Bot disconnected.' )
						self.robot = None
				else:
					try:
						self.robot = usbrobot.enumerate().next()
						logging.info( 'Bot connected.' )
						self.waves = 0
					except:
						self.robot = None
	
		self.connection.disconnect()

	def abort( self ):
		self._want_abort = True

	def send_wave( self ):
		self.connection.send(
				xmpp.protocol.Message(
						config['other'], config['action'] ) )
		if( self.robot ):
			self.robot.small_wave()

	def receive_wave( self ):
		if( self.robot ):
			self.robot.wave()
		else:
			self.waves += 1

	def message_handler( self, session, message ):
		if( message.getBody() == config['action'] ):
			self.receive_wave()


if __name__ == '__main__':

	j = JabberCommunicator()
	j.start()

	try:
		while( True ):
			time.sleep( 1 )
	finally:
		j.abort()

