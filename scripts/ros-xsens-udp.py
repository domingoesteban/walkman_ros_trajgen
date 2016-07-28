import socket
import struct

import rospy
from std_msgs import msg as stdMsg
from geometry_msgs import msg as geoMsg
from visualization_msgs import msg as visMsg

import operator
import math

# f = float = 4bytes
# i = integer = 4bytes
def getValue(data, type_byte='f', size=1):
   data = data[::-1]
   if size==1: return struct.unpack(type_byte, data)[0]
   return struct.unpack(type_byte*size, data)[::-1]

def header(data, render=True):
   if render:
      print '\n\n##################################'
      print 'ID string: ', data[0:6]
      print 'Sample counter: ', struct.unpack('I', data[6:10][::-1])[0]
      print 'Datagram counter: ', struct.unpack('B', data[10])[0]
      print 'Nb of items: ', struct.unpack('B', data[11])[0]
      print 'Time code: ', struct.unpack('I', data[12:16][::-1])[0]
      print 'Character ID: ', struct.unpack('B', data[16])[0]
      print 'Future use: ', data[17:]
      print '##################################'
   return data[4:6]

class Node:

   def __init__(self, IP, PORT):
      print "Creating Node"

      # UDP: SOCK_DGRAM | TCP: SOCK_STREAM
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      self.sock.bind( (IP, PORT) )

      # Segments ID - Name
      self.segments = [None, 'Pelvis', 'L5', 'L3', 'T12', 'T8', 'Neck', 'Head',
         'Right Shoulder', 'Right Upper Arm', 'Right Forearm', 'Right Hand',
         'Left Shoulder', 'Left Upper Arm', 'Left Forearm', 'Left Hand',
         'Right Upper Leg', 'Right Lower Leg', 'Right Foot', 'Right Toe',
         'Left Upper Leg', 'Left Lower Leg', 'Left Foot', 'Left Toe',
         'Prop1', 'Prop2', 'Prop3', 'Prop4']

      # ROS Publishers
      self.neckPitch = rospy.Publisher("/bigman/NeckPitchj_position_controller/command", stdMsg.Float64, queue_size=10)
      self.neckYaw = rospy.Publisher("/bigman/NeckYawj_position_controller/command", stdMsg.Float64, queue_size=10)

      self.waistLat = rospy.Publisher("/bigman/WaistLat_position_controller/command", stdMsg.Float64, queue_size=10)
      self.waistSag = rospy.Publisher("/bigman/WaistSag_position_controller/command", stdMsg.Float64, queue_size=10)
      self.waistYaw = rospy.Publisher("/bigman/WaistYaw_position_controller/command", stdMsg.Float64, queue_size=10)

      self.shoulderLLat = rospy.Publisher("/bigman/LShLat_position_controller/command", stdMsg.Float64, queue_size=10)
      self.shoulderLSag = rospy.Publisher("/bigman/LShSag_position_controller/command", stdMsg.Float64, queue_size=10)
      self.shoulderLYaw = rospy.Publisher("/bigman/LShYaw_position_controller/command", stdMsg.Float64, queue_size=10)
      self.shoulderRLat = rospy.Publisher("/bigman/RShLat_position_controller/command", stdMsg.Float64, queue_size=10)
      self.shoulderRSag = rospy.Publisher("/bigman/RShSag_position_controller/command", stdMsg.Float64, queue_size=10)
      self.shoulderRYaw = rospy.Publisher("/bigman/RShYaw_position_controller/command", stdMsg.Float64, queue_size=10)
      
      self.elbowL = rospy.Publisher("/bigman/LElbj_position_controller/command", stdMsg.Float64, queue_size=10)
      self.elbowR = rospy.Publisher("/bigman/RElbj_position_controller/command", stdMsg.Float64, queue_size=10)

      self.forearmL = rospy.Publisher("/bigman/LForearmPlate_position_controller/command", stdMsg.Float64, queue_size=10)
      self.forearmR = rospy.Publisher("/bigman/RForearmPlate_position_controller/command", stdMsg.Float64, queue_size=10)

      self.wristL1 = rospy.Publisher("/bigman/LWrj1_position_controller/command", stdMsg.Float64, queue_size=10)
      self.wristL2 = rospy.Publisher("/bigman/LWrj2_position_controller/command", stdMsg.Float64, queue_size=10)
      self.wristR1 = rospy.Publisher("/bigman/RWrj1_position_controller/command", stdMsg.Float64, queue_size=10)
      self.wristR2 = rospy.Publisher("/bigman/RWrj2_position_controller/command", stdMsg.Float64, queue_size=10)
      
      self.hipLLat = rospy.Publisher("/bigman/LHipLat_position_controller/command", stdMsg.Float64, queue_size=10)
      self.hipLSag = rospy.Publisher("/bigman/LHipSag_position_controller/command", stdMsg.Float64, queue_size=10)
      self.hipLYaw = rospy.Publisher("/bigman/LHipYaw_position_controller/command", stdMsg.Float64, queue_size=10)
      self.hipRLat = rospy.Publisher("/bigman/RHipLat_position_controller/command", stdMsg.Float64, queue_size=10)
      self.hipRSag = rospy.Publisher("/bigman/RHipSag_position_controller/command", stdMsg.Float64, queue_size=10)
      self.hipRYaw = rospy.Publisher("/bigman/RHipYaw_position_controller/command", stdMsg.Float64, queue_size=10)

      self.kneeLSag = rospy.Publisher("/bigman/LKneeSag_position_controller/command", stdMsg.Float64, queue_size=10)
      self.kneeRSag = rospy.Publisher("/bigman/RKneeSag_position_controller/command", stdMsg.Float64, queue_size=10)

      self.ankleLLat = rospy.Publisher("/bigman/LAnkLat_position_controller/command", stdMsg.Float64, queue_size=10)
      self.ankleLSag = rospy.Publisher("/bigman/LAnkSag_position_controller/command", stdMsg.Float64, queue_size=10)
      self.ankleRLat = rospy.Publisher("/bigman/RAnkLat_position_controller/command", stdMsg.Float64, queue_size=10)
      self.ankleRSag = rospy.Publisher("/bigman/RAnkSag_position_controller/command", stdMsg.Float64, queue_size=10)

      self.com = rospy.Publisher("/bigman/CoM", geoMsg.PointStamped, queue_size=10)
      self.marker_com = rospy.Publisher("/bigman/visual_CoM", visMsg.Marker, queue_size=1)


   def SetComMarkerProperty(self, frame_id, ns, id, shape, scale, pos):
      self.com_marker = visMsg.Marker(header = stdMsg.Header(frame_id=frame_id, stamp=rospy.Time.now()),
            ns = ns, id = id, type = shape, action = visMsg.Marker.ADD,
            pose = geoMsg.Pose(position=pos, orientation=geoMsg.Quaternion(0.0, 0.0, 0.0, 1.0)),
            scale = geoMsg.Vector3(1.0*scale, 1.0*scale, 1.0*scale), color = stdMsg.ColorRGBA(0.0, 1.0, 0.0, 1.0),
            lifetime = rospy.Duration())
     
   
   def run(self):
      #rate = rospy.Rate(50)
      print "Running"
      while not rospy.is_shutdown():
         data, addr = self.sock.recvfrom(1520)
         type_data = header(data[:24], render=False)

         start = 24
         end = len(data)
         #print "Length of data: ", len(data)

         if type_data == '01': # Euler (28bytes)
            for i in xrange(start, end, 28):
               print "Segment ID: ", self.segments[getValue(data[i:i+4], 'i', 1)]
               print "Segment Position: ", getValues(data[i+4:i+16], 'f', 3)
               print "Segment Rotation: ", getValues(data[i+16:i+28], 'f', 3)
               
         elif type_data == '02': # Quaternion (32bytes)
            for i in xrange(start, end, 32):
               print "Segment ID: ", self.segments[getValue(data[i:i+4], 'i', 1)]
               print "Segment Position: ", getValues(data[i+4:i+16], 'f', 3)
               print "Segment Rotation Quaternion: ", getValues(data[i+16:i+32], 'f', 4)

         elif type_data == '03': # Point Position Data (16bytes)
            for i in xrange(start, end, 16):
               print "Point ID: ", getValue(data[i:i+4], 'i', 1)
               print "Point Position: ", getValues(data[i+4:i+16], 'f', 3)

         elif type_data == '13': # Scale information
            print "Nb of segments: ", getValue(data[24:28], 'I', 1)
            # TODO
            print "Rest: ", data[28:]

         elif type_data == '20': # Joint Angles (20bytes)
            for i in xrange(start, end, 20):
               parentId = getValue(data[i:i+4], 'i', 1)/256
               childId = getValue(data[i+4:i+8], 'i', 1)/256

               # instead of if and else, I could use a dic
               if (parentId == 1) and (childId == 2): # pelvis and L5
                  rotL = [getValue(data[i+8:i+20], 'f', 3)]

               elif (parentId == 2) and (childId == 3): # L5 and L3
                  rotL.append(getValue(data[i+8:i+20], 'f', 3))

               elif (parentId == 3) and (childId == 4): # L3 and T12
                  rotL.append(getValue(data[i+8:i+20], 'f', 3))

               elif (parentId == 4) and (childId == 5): # T12 and T8
                  rotL.append(getValue(data[i+8:i+20], 'f', 3))
                  rot = tuple(map(sum, zip(*rotL)))
                  self.waistLat.publish(stdMsg.Float64(math.radians(rot[0])))
                  self.waistYaw.publish(stdMsg.Float64(math.radians(rot[1])))
                  self.waistSag.publish(stdMsg.Float64(math.radians(rot[2])))

               elif (parentId == 5) and (childId == 6): # T8 and Neck
                  rot1 = getValue(data[i+8:i+20], 'f', 3)

               elif (parentId == 6) and (childId == 7): # Neck and Head
                  rot2 = getValue(data[i+8:i+20], 'f', 3)
                  totRot = tuple(map(operator.add, rot1, rot2))
                  yaw = math.radians(totRot[1])
                  pitch = math.radians(totRot[2])
                  self.neckYaw.publish(stdMsg.Float64(yaw))
                  self.neckPitch.publish(stdMsg.Float64(pitch))

               elif (parentId == 12) and (childId == 13): # left shoulder & upper arm
                  rot = getValue(data[i+8:i+20], 'f', 3)
                  self.shoulderLLat.publish(stdMsg.Float64(math.radians(rot[0])))
                  self.shoulderLYaw.publish(stdMsg.Float64(-math.radians(rot[1])))
                  self.shoulderLSag.publish(stdMsg.Float64(-math.radians(rot[2])))

               elif (parentId == 8) and (childId == 9): # right shoulder & upper arm
                  rot = getValue(data[i+8:i+20], 'f', 3)
                  self.shoulderRLat.publish(stdMsg.Float64(-math.radians(rot[0])))
                  self.shoulderRYaw.publish(stdMsg.Float64(math.radians(rot[1])))
                  self.shoulderRSag.publish(stdMsg.Float64(-math.radians(rot[2])))

               elif (parentId == 13) and (childId == 14): # left upper arm & forearm (elbow)
                  rotLUF = getValue(data[i+8:i+20], 'f', 3)
                  self.elbowL.publish(stdMsg.Float64(-math.radians(rotLUF[2])))

               elif (parentId == 9) and (childId == 10): # right upper arm & forearm (elbow)
                  rotRUF = getValue(data[i+8:i+20], 'f', 3)
                  self.elbowR.publish(stdMsg.Float64(-math.radians(rotRUF[2])))

               elif (parentId == 14) and (childId == 15): # left forearm & hand (wrist)
                  rot = getValue(data[i+8:i+20], 'f', 3)
                  totRot = tuple(map(operator.add, rotLUF, rot))
                  self.forearmL.publish(stdMsg.Float64(-math.radians(totRot[1]) + math.pi/2))
                  self.wristL1.publish(stdMsg.Float64(-math.radians(rot[0])))
                  self.wristL2.publish(stdMsg.Float64(-math.radians(rot[2]))) # Lat

               elif (parentId == 10) and (childId == 11): # right forearm & hand (wrist)
                  rot = getValue(data[i+8:i+20], 'f', 3)
                  totRot = tuple(map(operator.add, rotRUF, rot))
                  self.forearmR.publish(stdMsg.Float64(math.radians(totRot[1]) - math.pi/2))
                  self.wristR1.publish(stdMsg.Float64(-math.radians(rot[0])))
                  self.wristR2.publish(stdMsg.Float64(math.radians(rot[2]))) # Lat

               elif (parentId == 1) and (childId == 20): # pelvis & left upper leg (hip)
                  rot = getValue(data[i+8:i+20], 'f', 3)
                  self.hipLLat.publish(stdMsg.Float64(math.radians(rot[0])))
                  self.hipLYaw.publish(stdMsg.Float64(-math.radians(rot[1])))
                  self.hipLSag.publish(stdMsg.Float64(-math.radians(rot[2])))

               elif (parentId == 1) and (childId == 16): # pelvis & right upper leg (hip)
                  rot = getValue(data[i+8:i+20], 'f', 3)
                  self.hipRLat.publish(stdMsg.Float64(-math.radians(rot[0])))
                  self.hipRYaw.publish(stdMsg.Float64(math.radians(rot[1])))
                  self.hipRSag.publish(stdMsg.Float64(-math.radians(rot[2])))

               elif (parentId == 20) and (childId == 21): # left upper leg & lower leg (knee)
                  rot = getValue(data[i+8:i+20], 'f', 3)
                  self.kneeLSag.publish(stdMsg.Float64(math.radians(rot[2])))

               elif (parentId == 16) and (childId == 17): # right upper leg & lower leg (knee)
                  rot = getValue(data[i+8:i+20], 'f', 3)
                  self.kneeRSag.publish(stdMsg.Float64(math.radians(rot[2])))

               elif (parentId == 21) and (childId == 22): # left lower leg & foot (ankle)
                  rot = getValue(data[i+8:i+20], 'f', 3)
                  self.ankleLLat.publish(stdMsg.Float64(math.radians(rot[0])))
                  self.ankleLSag.publish(stdMsg.Float64(-math.radians(rot[2])))
                  
               elif (parentId == 17) and (childId == 18): # right lower leg & foot (ankle)
                  rot = getValue(data[i+8:i+20], 'f', 3)
                  self.ankleRLat.publish(stdMsg.Float64(-math.radians(rot[0])))
                  self.ankleRSag.publish(stdMsg.Float64(-math.radians(rot[2])))
       
               #print "Point ID of parent: ", parentId
               #print "Point ID of child: ", childId
               #print "Floating point rotation: ", getValue(data[i+8:i+20], 'f', 3)
               
         elif type_data == '21': # Linear Segment Kinematics (40bytes)
            for i in xrange(start, end, 40):
               print "Segment ID: ", self.segments[getValue(data[i:i+4], 'i', 1)]
               print "Segment Position: ", getValue(data[i+4:i+16], 'f', 3)
               print "Segment Global Velocity: ", getValue(data[i+16:i+28], 'f', 3)
               print "Segment Global Acceleration: ", getValue(data[i+28:i+40], 'f', 3)

         elif type_data == '22': # Angular Segment Kinematics (44bytes)
            for i in xrange(start, end, 44):
               print "Segment ID: ", self.segments[getValue(data[i:i+4], 'i', 1)]
               print "Segment Rotation Quaternion: ", getValue(data[i+4:i+20], 'f', 4)
               print "Segment Global Angular Velocity: ", getValue(data[i+20:i+32], 'f', 3)
               print "Segment Global Angular Acceleration: ", getValue(data[i+32:i+44], 'f', 3)

         elif type_data == '23': # Motion Tracker Kinematics (68bytes)
            for i in xrange(start, end, 68):
               print "Segment ID: ", self.segments[getValue(data[i:i+4], 'i', 1)]
               print "Global Rotation (Quaternion): ", getValue(data[i+4:i+20], 'f', 4)
               print "Global Free Acceleration: ", getValue(data[i+20:i+32], 'f', 3)
               print "Segment Local Acceleration: ", getValue(data[i+32:i+44], 'f', 3)
               print "Segment Local Angular Velocity: ", getValue(data[i+44:i+56], 'f', 3)
               print "Segment Local Magnetic Field: ", getValue(data[i+56:i+68], 'f', 3)

         elif type_data == '24': # Center of Mass (12bytes)
            com = getValue(data[24:36], 'f', 3)
            
            # update com marker inside rviz
            self.SetComMarkerProperty("base_link", "com_namespace", 0, visMsg.Marker.SPHERE, 0.1, com)
            self.marker_com.publish(self.com_marker)
        
            # publish com msg
            com_msg = geoMsg.PointStamped(header=stdMsg.Header(stamp=rospy.Time.now()), point=com)
            self.com.publish(com_msg)
            #print "Center of Mass Position: ", com

         elif type_data == '25': # Time Code (12bytes)
            print "Time Code: ", data[24:36]
         else:
            raise NotImplementedError
    
      #rate.sleep()

if __name__ == "__main__":
   rospy.init_node('Xsens')
   node = Node('10.255.41.105', 2004)
   node.run()
   rospy.spin()

