'''
.. module:: optitrack
   :synopsis: Module for handling and retrieving OptiTrack positioning data.

.. module original author:: Cristian Ioan Vasile <cvasile@bu.edu>
   module modified by :: Guang Yang <gyang101@bu.edu>, Zijian Wang
'''
'''
    Module for handling and retrieving OptiTrack positioning data.
    Copyright (C) 2013  Cristian Ioan Vasile <cvasile@bu.edu>
    Hybrid and Networked Systems (HyNeSs) Laboratory, Boston University

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import socket
import struct
import sys
import time

from util import getLocalIP
from util import rot2ZYXeuler, quaternion2rot
from parseNatNet import RigidBody, parseNatNet



class OptiTrackInterface(object):
    '''
    classdocs
    '''
    
    def __init__(self, localIP=None, buffersize=2**12,
                 multicastIP='239.255.42.99', multicastPort=1511):
        '''
        Constructor
        '''
        self.__multicastIP = multicastIP
        if localIP:
            self.__localIP = localIP
        else:
            self.__localIP = getLocalIP()
        
        self.__port = multicastPort
        self.__buffer = bytearray(buffersize)
        self.buffersize = buffersize
        
        # TODO: define tracked objects and origin
        self.origin = RigidBody()
        
        self.setup()

        print "OptiTrack interface initialized, self IP address = ", self.__localIP
    
    def __del__(self):
        '''
        Destructor -- closes the socket
        '''
        self.__socket.close()
        
    def _close(self): # FIXME: delete this --- temp solution
        self.__socket.close()
    
    def setup(self):
        """
        Creates a socket, sets the necessary options on it, then binds it.
        """
         
        # create a UDP socket
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
           
        # allow reuse of addresses
        self.__socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # request membership to multicast group
#         msgReq = struct.pack("4sl", socket.inet_aton(self.__multicastIP),
#                              socket.INADDR_ANY)
        msgReq = struct.pack("4s4s", socket.inet_aton(self.__multicastIP),
                             socket.inet_aton(self.__localIP))
         
        self.__socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                 msgReq)

        # bind socket to multicast port
        # NOTE: for Linux Ubuntu/Mac you have to bind to all accessible addresses in
        # order to receive something
        # TODO: test on Windows -- it seems on Windows you have bind to actual
        # local address to receive something
        if sys.platform.startswith('win32'):
            self.__socket.bind((self.__localIP, self.__port))
        else:
            self.__socket.bind(('', self.__port))
        
#-------------------------------------------------------------------------------

    def get_pos(self, obj_id):
        '''
        Get the 6-DOF pose of the tracked object. To use:
        (x,y,z,pitch,roll,yaw) = self.get_pos(obj_id)
        '''

        # Receive data from socket
        msg = self.__socket.recv(4096)
        
        # Parse data 
        markerSets, rigidBodies = parseNatNet(msg)
               
        # Retrieve the info of the specified rigid body 
        rigidBody = rigidBodies[obj_id]

        # Calculate Euler angles
        euler = rot2ZYXeuler(quaternion2rot(rigidBody.SE3[3:])) # pitch,row , yaw

        # output format: [x,y,z,pitch,roll,yaw]. Positive direction: CCW -> Pitch(+down), Roll(+bank right)
        return (rigidBody.SE3[2], rigidBody.SE3[0], rigidBody.SE3[1], euler[0], euler[1], euler[2]) #(x,y,z,pitch,roll,yaw)


    def PrintTrackInfo(self, obj_id):
        '''
        TBD
        '''
        (x,y,z,pitch,roll,yaw) = self.get_pos(obj_id)
        print("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f" % (x,y,z,pitch,roll,yaw) )



#if __name__ == '__main__':
    #test OptiTrack interface
    
#    opti = OptiTrackInterface()
#    opti.PrintTrackInfo(1)


