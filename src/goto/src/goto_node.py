#!/usr/bin/env python

from __future__ import print_function
import readline
import rospy
import time
import numpy as np
import os
import os.path as osp
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pitranger.srv import *
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, CompressedImage
from PIL import Image
import piexif
import nav
import io

BRINK_LIMIT_M = 0.4
ROBOT_VELOCITY = 0.12

image_count = 0

class RobotExecutive:
  def __init__(self):
    self.image_path = None
    self.image_exposures = None
    self.image_pans = None
    self.image_tilts = None
    self.default_pan = None
    self.default_tilt = None

    # Count image capture locations
    self.capture_idx = 0

    # Capture odom messages and use them to set the robot state.
    self.state = None
    self.odom_sub = rospy.Subscriber('/whereami/odom/', Odometry, self.odom_callback)
    print("Waiting for odometry...")
    self.wait_for_odom()

    # Use GPS fix to write exif location into images.
    self.gps_fix = None
    self.gps_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
    print("Waiting for gps fix...")
    self.wait_for_gps()

    # Watch the brinkmanship range estimate to safeguard while driving.
    self.brink_range = None
    self.brink_sub = rospy.Subscriber('/brink/out/range', Float64, self.brink_callback)
    print("Waiting for brinkmanship...")
    self.wait_for_brink()

    # Publish motor commands.
    self.twist_pub = rospy.Publisher('/teleop/out/twist_cmd', Twist, queue_size=10)
    # Publish pitcam images.
    self.pitcam_pub = rospy.Publisher('/pitcam/image/compressed', CompressedImage, queue_size=10)
    self.pitcam_thumb_pub = rospy.Publisher('/pitcam/thumbnail/compressed', CompressedImage, queue_size=10)
    print("HERE WE GO!")

  def is_initialized(self):
      return (self.image_path is not None and self.image_exposures is not None and \
              self.image_pans is not None and self.image_tilts is not None and \
              self.default_pan is not None and self.default_tilt is not None)

  def set_image_path(self, path):
    if not os.path.exists(path):
      os.mkdir(path)
    self.image_path = path

  def set_image_exposures(self, exposures):
    self.image_exposures = exposures

  def set_image_pans(self, pans):
    self.image_pans = pans

  def set_image_tilts(self, tilts):
    self.image_tilts = tilts

  def set_default_pan(self, p):
    print("SET DEFAULT PAN")
    self.default_pan = p
    if self.default_tilt is not None:
      self._set_pan_tilt(self.default_pan, self.default_tilt)

  def set_default_tilt(self, t):
    print("SET DEFAULT TILT")
    self.default_tilt = t
    if self.default_pan is not None:
      self._set_pan_tilt(self.default_pan, self.default_tilt)

  def odom_callback(self, msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ori = msg.pose.pose.orientation
    q = np.array([ori.x, ori.y, ori.z, ori.w])
    _,_,yaw = euler_from_quaternion(q)
    self.state = np.array([x, y, yaw])

  def gps_callback(self, msg):
    self.gps_fix = msg

  def brink_callback(self, msg):
    self.brink_range = msg.data

  def wait_for_odom(self):
    while self.state is None:
      time.sleep(0.1)

  def wait_for_gps(self):
    while self.gps_fix is None:
      time.sleep(0.1)

  def wait_for_brink(self):
    while self.brink_range is None:
      time.sleep(0.1)

  def goto_absolute(self, goal_x, goal_y, brink_en, dxy=0.5, dh=2.0*np.pi/180.0):
    while(not rospy.is_shutdown()):
      p0 = nav.Pose(self.state[0], self.state[1], self.state[2])
      p1 = nav.Pose(goal_x, goal_y, 0.0)
      g = p1.xy-p0.xy 
      theta = np.arctan2(g[1],g[0])

      # If you are close enough to the goal...
      if( np.linalg.norm(g) <= dxy ):
        # REORIENT
        #print("REORIENT")
        #self.face(p1.h)
        self.twist_pub.publish(Twist())
        return
      else:
        dh_fwd = nav.subtract_angles(p0.h, theta)
        dh_rev = nav.subtract_angles(p0.h+np.pi, theta)

        # Should I drive forward? Or backward?
        # NOTE(JORDAN): If brinkmanship is enabled, you MUST drive forward.
        if brink_en or abs(dh_fwd) < abs(dh_rev):
          # Drive forward...
          if abs(dh_fwd) > 10*np.pi/180.0:
            # ORIENT
            #print("ORIENT FWD")
            self.face(theta)
          else:
            # Stop if you are at the brink!
            if(brink_en):
              print("BRINK: {}".format(self.brink_range))
            if(brink_en and 0.2 < self.brink_range < BRINK_LIMIT_M):
              self.twist_pub.publish(Twist())
              break

            # DRIVE FWD
            if np.sin(dh_fwd) == 0.0:
              R = 1e8
            else:
              R = np.linalg.norm(g) / (2*np.sin(dh_fwd))
            #print("DRIVING FORWARD!")
            msg = Twist()
            if( brink_en ):
              msg.linear.x =  0.03
            else:
              msg.linear.x =  ROBOT_VELOCITY
            msg.angular.z = msg.linear.x / R
            self.twist_pub.publish(msg)
            #print("DRIVE FWD")
        else:
          # Drive backward...
          if abs(dh_rev) > 10*np.pi/180.0:
            # ORIENT
            #print("ORIENT REV")
            self.face(theta+np.pi)
          else:
            # DRIVE REV
            if np.sin(dh_rev) == 0.0:
              R = 1e8
            else:
              R = np.linalg.norm(g) / (2*np.sin(dh_rev))
            #print("DRIVING BACKWARD!")
            msg = Twist()
            msg.linear.x = -ROBOT_VELOCITY
            msg.angular.z = ROBOT_VELOCITY / R
            self.twist_pub.publish(msg)
            #print("DRIVE REV")
      time.sleep(0.05)

  def goto_relative(self, goal_x, goal_y, brink_en):
    # Rotate the relative vector from the robot frame to the ENU frame centered on the robot.
    vec = np.array([goal_x, goal_y])
    c = np.cos(self.state[2]); s = np.sin(self.state[2]);
    M = np.array([[c,-s],[s,c]])
    vec = M.dot(vec)

    # Translate the vector into the world frame.
    new_x = self.state[0] + vec[0]
    new_y = self.state[1] + vec[1]
    return self.goto_absolute(new_x, new_y, brink_en)

  def face(self, goal_h):
    # Rotate to face the right way.
    while(not rospy.is_shutdown()):
      dh = nav.subtract_angles(self.state[2], goal_h)
      if abs(dh*180./np.pi) <= 1.0:
        break
      msg = Twist()
      msg.angular.z = 0.1*np.sign(dh)
      self.twist_pub.publish(msg)
      time.sleep(0.1)
    # Stop!
    self.twist_pub.publish(Twist())

  def point(self, point_x, point_y):
    h = np.arctan2(point_y-self.state[1], point_x-self.state[0])
    self.face(h)

  def _set_pan_tilt(self, p, t):
    rospy.wait_for_service('/pitcam/set_pan_tilt')
    try:
      set_pt = rospy.ServiceProxy('/pitcam/set_pan_tilt', SetPanTilt)
      set_pt(p, t)
    except rospy.ServiceException as e:
      print("ERROR: SetPanTilt service call failed: %s"%e)

  def _capture_image(self, e):
    print("CAPTURING IMAGE")
    rospy.wait_for_service('/pitcam/capture')
    try:
      capture = rospy.ServiceProxy('/pitcam/capture', PitCamCapture)
      resp = capture(e)
      return resp
    except rospy.ServiceException as e:
      print("ERROR: PitCamCapture service call failed: %s"%e)
    return None

  def capture(self):
    pans = self.image_pans
    for t in self.image_tilts:
      # Loop over exposure bracket
      pans.reverse()
      for p in pans:
        # Aim the camera and wait for it to stabilize.
        self._set_pan_tilt(p, t)
        time.sleep(0.5)

        for e in self.image_exposures:
          for n in range(1):

            # Check if ros is dead.
            if rospy.is_shutdown():
              return 

            # Capture an image!
            img_data = self._capture_image(e)
            if img_data is None:
              print("Failed to capture image!")
              continue
            print("IMAGE CAPTURED AT PAN {} TILT {} EXP {}".format(img_data.pan_deg, img_data.tilt_deg, img_data.exposure_us))

            if img_data is None:
              print("GOTO ERROR: Failed to capture image!")
            elif img_data.image.encoding != "RGB8":
              print("GOTO ERROR: Pitcam image encoding {} not recognized!".format(img_data.image.encoding))
            else:
              img = Image.frombytes('RGB', (img_data.image.width,img_data.image.height), img_data.image.data, 'raw')
              img_name = "v{:03}_t{:+03}_p{:+03}_e{:09}.jpg".format(self.capture_idx, t, p, e, n)
              img_path = osp.join(self.image_path, img_name)
              print(img_name)
              
              # Write metadata into exif
              zeroth_ifd = {piexif.ImageIFD.Make: "FLIR BFS-PGE-200S6C-C",
                            piexif.ImageIFD.XResolution: (96, 1),
                            piexif.ImageIFD.YResolution: (96, 1),
                            piexif.ImageIFD.Software: "pitranger"
              }
              exif_ifd = {piexif.ExifIFD.ExifVersion: b'\x02\x00\x00\x00',
                          piexif.ExifIFD.LensMake: "Computar V2520-MPZ",
                          piexif.ExifIFD.ExposureTime: (int(e), 1000000),
                          piexif.ExifIFD.FocalLength: (25, 1),
                          piexif.ExifIFD.Sharpness: 0,
                          piexif.ExifIFD.Gamma: (18,10),
                          piexif.ExifIFD.MakerNote: "Jordan is the coolest!",
                          piexif.ExifIFD.FocalPlaneResolutionUnit: 4, # 4 => mm
                          piexif.ExifIFD.FocalPlaneXResolution: (416755,1000),
                          piexif.ExifIFD.FocalPlaneYResolution: (416438,1000),
              }
              gps_ifd = {piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
                         piexif.GPSIFD.GPSAltitudeRef: 0,
                         piexif.GPSIFD.GPSLatitudeRef: "N",
                         piexif.GPSIFD.GPSLongitudeRef: "W",
                         piexif.GPSIFD.GPSLatitude:   [int(abs(self.gps_fix.latitude)*100000), 100000],
                         piexif.GPSIFD.GPSLongitude:  [int(abs(self.gps_fix.longitude)*100000), 100000]
              }
              exif_dict = {"0th":zeroth_ifd,
                           "Exif":exif_ifd,
                           "GPS":gps_ifd
              }
              exif_bytes = piexif.dump(exif_dict)

              # Save image as jpg with exif
              img.save(img_path, exif=exif_bytes)

              # Get the raw image bytes after jpg compression
              def get_jpg_bytes(img, exif):
                img_byte_arr = io.BytesIO()
                img_bytes = img.save(img_byte_arr, format='JPEG', exif=exif_bytes)
                img_byte_arr = img_byte_arr.getvalue()
                return img_byte_arr

              # Publish the image.
              msg = CompressedImage()
              global image_count
              msg.header.seq = image_count
              msg.header.stamp = rospy.get_rostime()
              msg.header.frame_id = 'pitcam_optical_frame'
              msg.format = 'jpeg'
              msg.data = get_jpg_bytes(img, exif_bytes)
              self.pitcam_pub.publish(msg)

              # Publish its thumbnail.
              msg = CompressedImage()
              msg.header.seq = image_count
              msg.header.stamp = rospy.get_rostime()
              msg.header.frame_id = 'pitcam_optical_frame'
              msg.format = 'jpeg'
              img.thumbnail((256,256), Image.ANTIALIAS)
              msg.data = get_jpg_bytes(img, exif_bytes)
              self.pitcam_thumb_pub.publish(msg)
              image_count += 1

    # Return the camera to its default position.
    self._set_pan_tilt(self.default_pan, self.default_tilt)
    # Count captures for organizing images.
    self.capture_idx += 1

  def fake_capture(self):
    pans = self.image_pans
    for t in self.image_tilts:
      # Loop over exposure bracket
      pans.reverse()
      for p in pans:
        # Aim the camera and wait for it to stabilize.
        self._set_pan_tilt(p, t)
        time.sleep(1.0)

    # Return the camera to its default position.
    self._set_pan_tilt(self.default_pan, self.default_tilt)
    # Count captures for organizing images.
    self.capture_idx += 1

  def slow_capture(self):
    pans = self.image_pans
    for t in self.image_tilts:

      # Once per tilt, set the autoexposure
      self._set_pan_tilt(0, t)
      print("Performing autoexposure...")
      resp = self._capture_image(0)

      if resp is None:
        print("Failed to capture image!")
        continue

      ae = resp.exposure_us
      self.image_exposures = np.array([
        int(ae/2.0), int(ae), int(ae*2.0)])
      print("Exposure Bracket: {}".format(self.image_exposures))

      # Pan the camera
      pans.reverse()
      for p in pans:
        # Aim the camera and wait for it to stabilize.
        self._set_pan_tilt(p, t)
        time.sleep(0.5)

        # Loop over exposure bracket
        for e in self.image_exposures:
          for n in range(1):

            # Check if ros is dead.
            if rospy.is_shutdown():
              return 

            # Capture an image!
            img_data = self._capture_image(e)
            if img_data is None:
              print("Failed to capture image!")
              continue
            print("IMAGE CAPTURED AT PAN {} TILT {} EXP {}".format(img_data.pan_deg, img_data.tilt_deg, img_data.exposure_us))

            if img_data is None:
              print("GOTO ERROR: Failed to capture image!")
            elif img_data.image.encoding != "RGB8":
              print("GOTO ERROR: Pitcam image encoding {} not recognized!".format(img_data.image.encoding))
            else:
              img = Image.frombytes('RGB', (img_data.image.width,img_data.image.height), img_data.image.data, 'raw')
              img_name = "v{:03}_t{:+03}_p{:+03}_e{:09}.jpg".format(self.capture_idx, t, p, e, n)
              img_path = osp.join(self.image_path, img_name)
              print(img_name)
              
              # Write metadata into exif
              zeroth_ifd = {piexif.ImageIFD.Make: "FLIR BFS-PGE-200S6C-C",
                            piexif.ImageIFD.XResolution: (96, 1),
                            piexif.ImageIFD.YResolution: (96, 1),
                            piexif.ImageIFD.Software: "pitranger"
              }
              exif_ifd = {piexif.ExifIFD.ExifVersion: b'\x02\x00\x00\x00',
                          piexif.ExifIFD.LensMake: "Computar V2520-MPZ",
                          piexif.ExifIFD.ExposureTime: (int(e), 1000000),
                          piexif.ExifIFD.FocalLength: (25, 1),
                          piexif.ExifIFD.Sharpness: 0,
                          piexif.ExifIFD.Gamma: (18,10),
                          piexif.ExifIFD.MakerNote: "Jordan is the coolest!",
                          piexif.ExifIFD.FocalPlaneResolutionUnit: 4, # 4 => mm
                          piexif.ExifIFD.FocalPlaneXResolution: (416755,1000),
                          piexif.ExifIFD.FocalPlaneYResolution: (416438,1000),
              }
              gps_ifd = {piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
                         piexif.GPSIFD.GPSAltitudeRef: 0,
                         piexif.GPSIFD.GPSLatitudeRef: "N",
                         piexif.GPSIFD.GPSLongitudeRef: "W",
                         piexif.GPSIFD.GPSLatitude:   [int(abs(self.gps_fix.latitude)*100000), 100000],
                         piexif.GPSIFD.GPSLongitude:  [int(abs(self.gps_fix.longitude)*100000), 100000]
              }
              exif_dict = {"0th":zeroth_ifd,
                           "Exif":exif_ifd,
                           "GPS":gps_ifd
              }
              exif_bytes = piexif.dump(exif_dict)

              # Save image as jpg with exif
              img.save(img_path, exif=exif_bytes)

              # Get the raw image bytes after jpg compression
              def get_jpg_bytes(img, exif):
                img_byte_arr = io.BytesIO()
                img_bytes = img.save(img_byte_arr, format='JPEG', exif=exif_bytes)
                img_byte_arr = img_byte_arr.getvalue()
                return img_byte_arr

              # Publish the image.
              msg = CompressedImage()
              global image_count
              msg.header.seq = image_count
              msg.header.stamp = rospy.get_rostime()
              msg.header.frame_id = 'pitcam_optical_frame'
              msg.format = 'jpeg'
              msg.data = get_jpg_bytes(img, exif_bytes)
              self.pitcam_pub.publish(msg)

              # Publish its thumbnail.
              msg = CompressedImage()
              msg.header.seq = image_count
              msg.header.stamp = rospy.get_rostime()
              msg.header.frame_id = 'pitcam_optical_frame'
              msg.format = 'jpeg'
              img.thumbnail((256,256), Image.ANTIALIAS)
              msg.data = get_jpg_bytes(img, exif_bytes)
              self.pitcam_thumb_pub.publish(msg)
              image_count += 1

    # Return the camera to its default position.
    self._set_pan_tilt(self.default_pan, self.default_tilt)
    # Count captures for organizing images.
    self.capture_idx += 1


if __name__=="__main__":
  rospy.init_node('goto')

  mission_file_path = rospy.get_param('~mission')
  mission_file_path = osp.expanduser(mission_file_path)

  robot = RobotExecutive()

  with open(mission_file_path, 'r') as mission_file:
    print("Found mission file {}".format(mission_file_path))

    # Begin interpreting commands here!
    line_number = 0
    mission_file_lines = mission_file.readlines()

    while True:
      try:
        if line_number < len(mission_file_lines):
          command = mission_file_lines[line_number]
          line_number += 1
        else:
          print(">> ", end='')
          command = raw_input()

        if rospy.is_shutdown():
          break

        # Skip comment lines.
        comment_idx = command.find('#')
        if comment_idx != -1:
          command = command[0:comment_idx]

        command = command.strip().split(' ')
        command = [c.strip(',"') for c in command if c != '']

        # Skip blank lines.
        if len(command) == 0:
          continue

        # Use uppercase commands
        command[0] = command[0].upper()

        # Set image path.
        if command[0] == 'IMG_PATH':
          image_path = command[1]
          image_path = osp.expanduser(image_path)
          robot.set_image_path(image_path)
          continue

        # Set image exposures.
        if command[0] == 'EXP':
          image_exposures = [int(x) for x in command[1:]]
          robot.set_image_exposures(image_exposures)
          continue

        # Set pitcam pans.
        if command[0] == 'PAN':
          image_pans = [int(x) for x in command[1:]]
          robot.set_image_pans(image_pans)
          continue

        # Set pitcam tilts.
        if command[0] == 'TILT':
          image_tilts = [int(x) for x in command[1:]]
          robot.set_image_tilts(image_tilts)
          continue

        if command[0] == 'DEFAULT_PAN':
          robot.set_default_pan(int(command[1]))
          continue

        if command[0] == 'DEFAULT_TILT':
          robot.set_default_tilt(int(command[1]))
          continue

        # If you have made it here, you better be ready to go!
        if not robot.is_initialized():
          print("GOTO ERROR [Line {}]: Make sure you have set IMG_PATH, EXP, PAN, TILT, DEFAULT_PAN, and DEFAULT_TILT before you try to ABS, REL, or CAP!".format(line_number))
          break

        # Drive to an absolute pose.
        if command[0] == 'ABS':
          goal_x = float(command[1])
          goal_y = float(command[2])
          brink_en = command[3].upper() != 'BRINK_OFF'
          robot.goto_absolute(goal_x, goal_y, brink_en)
          continue

        # Drive to a relative pose.
        if command[0] == 'REL':
          goal_x = float(command[1])
          goal_y = float(command[2])
          brink_en = command[3].upper() != 'BRINK_OFF'
          robot.goto_relative(goal_x, goal_y, brink_en)
          continue

        # Turn in place.
        if command[0] == 'FACE':
          goal_h = np.pi/180.0 * float(command[1])
          robot.face(goal_h)
          continue

        # Turn toward another position.
        if command[0] == 'POINT':
          goal_x = float(command[1])
          goal_y = float(command[2])
          robot.point(goal_x, goal_y)
          continue

        # Turn toward another position.
        if command[0] == 'SET_PAN_TILT':
          p = float(command[1])
          t = float(command[2])
          robot._set_pan_tilt(p, t)
          continue

        # Capture pitcam images.
        if command[0] == 'CAP':
          robot.capture()
          continue

        # Capture pitcam images.
        if command[0] == 'SLOW_CAP':
          robot.slow_capture()
          continue

        # Pretend to capture pitcam images.
        if command[0] == 'FAKE':
          robot.fake_capture()
          continue

        # Capture pitcam images.
        if command[0] == 'STOP':
          robot.goto_relative(0.0, 0.0, False)
          continue

        # Pause for a number of seconds
        if command[0] == 'PAUSE':
          time.sleep(float(command[1]))
          continue

        # Capture pitcam images.
        if command[0] == 'WHERE':
          print('({:3f}, {:3f}, {:3f})'.format(
                robot.state[0], robot.state[1], robot.state[2]*180.0/np.pi))
          continue

        # Quit the interpreter.
        if command[0] == 'QUIT':
          robot.goto_relative(0.0, 0.0, False)
          break

        # Unrecognized command!
        print("ERROR: Unrecognized command \"{}\" on line {}. Skipping.".format(command[0], line_number))

      except Exception as e:
          print("ERROR: {}".format(e))
