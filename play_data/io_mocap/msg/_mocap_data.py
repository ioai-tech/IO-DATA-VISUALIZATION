# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from io_mocap/mocap_data.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class mocap_data(genpy.Message):
  _md5sum = "259d1c7eb3af4e861f9bbd145174f344"
  _type = "io_mocap/mocap_data"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
float32[4] joint_quaternion
float32[4] raw_quaternion
float32[4] world_quaternion
float32[3] mag
float32[3] accel
float32[3] gyro
float32[3] raw_mag
float32[3] raw_accel
float32[3] raw_gyro
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  __slots__ = ['header','joint_quaternion','raw_quaternion','world_quaternion','mag','accel','gyro','raw_mag','raw_accel','raw_gyro']
  _slot_types = ['std_msgs/Header','float32[4]','float32[4]','float32[4]','float32[3]','float32[3]','float32[3]','float32[3]','float32[3]','float32[3]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,joint_quaternion,raw_quaternion,world_quaternion,mag,accel,gyro,raw_mag,raw_accel,raw_gyro

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(mocap_data, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.joint_quaternion is None:
        self.joint_quaternion = [0.] * 4
      if self.raw_quaternion is None:
        self.raw_quaternion = [0.] * 4
      if self.world_quaternion is None:
        self.world_quaternion = [0.] * 4
      if self.mag is None:
        self.mag = [0.] * 3
      if self.accel is None:
        self.accel = [0.] * 3
      if self.gyro is None:
        self.gyro = [0.] * 3
      if self.raw_mag is None:
        self.raw_mag = [0.] * 3
      if self.raw_accel is None:
        self.raw_accel = [0.] * 3
      if self.raw_gyro is None:
        self.raw_gyro = [0.] * 3
    else:
      self.header = std_msgs.msg.Header()
      self.joint_quaternion = [0.] * 4
      self.raw_quaternion = [0.] * 4
      self.world_quaternion = [0.] * 4
      self.mag = [0.] * 3
      self.accel = [0.] * 3
      self.gyro = [0.] * 3
      self.raw_mag = [0.] * 3
      self.raw_accel = [0.] * 3
      self.raw_gyro = [0.] * 3

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      buff.write(_get_struct_4f().pack(*self.joint_quaternion))
      buff.write(_get_struct_4f().pack(*self.raw_quaternion))
      buff.write(_get_struct_4f().pack(*self.world_quaternion))
      buff.write(_get_struct_3f().pack(*self.mag))
      buff.write(_get_struct_3f().pack(*self.accel))
      buff.write(_get_struct_3f().pack(*self.gyro))
      buff.write(_get_struct_3f().pack(*self.raw_mag))
      buff.write(_get_struct_3f().pack(*self.raw_accel))
      buff.write(_get_struct_3f().pack(*self.raw_gyro))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 16
      self.joint_quaternion = _get_struct_4f().unpack(str[start:end])
      start = end
      end += 16
      self.raw_quaternion = _get_struct_4f().unpack(str[start:end])
      start = end
      end += 16
      self.world_quaternion = _get_struct_4f().unpack(str[start:end])
      start = end
      end += 12
      self.mag = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 12
      self.accel = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 12
      self.gyro = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 12
      self.raw_mag = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 12
      self.raw_accel = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 12
      self.raw_gyro = _get_struct_3f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      buff.write(self.joint_quaternion.tostring())
      buff.write(self.raw_quaternion.tostring())
      buff.write(self.world_quaternion.tostring())
      buff.write(self.mag.tostring())
      buff.write(self.accel.tostring())
      buff.write(self.gyro.tostring())
      buff.write(self.raw_mag.tostring())
      buff.write(self.raw_accel.tostring())
      buff.write(self.raw_gyro.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 16
      self.joint_quaternion = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=4)
      start = end
      end += 16
      self.raw_quaternion = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=4)
      start = end
      end += 16
      self.world_quaternion = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=4)
      start = end
      end += 12
      self.mag = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.accel = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.gyro = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.raw_mag = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.raw_accel = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.raw_gyro = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f
_struct_4f = None
def _get_struct_4f():
    global _struct_4f
    if _struct_4f is None:
        _struct_4f = struct.Struct("<4f")
    return _struct_4f
