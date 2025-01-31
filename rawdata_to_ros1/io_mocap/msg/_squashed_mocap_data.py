# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from io_mocap/squashed_mocap_data.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import io_mocap.msg
import std_msgs.msg

class squashed_mocap_data(genpy.Message):
  _md5sum = "acba46ce9e87b7a125daa9e77791e9fe"
  _type = "io_mocap/squashed_mocap_data"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
mocap_data[] data
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

================================================================================
MSG: io_mocap/mocap_data
Header header
float32[4] joint_quaternion
float32[4] raw_quaternion
float32[4] world_quaternion
float32[3] mag
float32[3] accel
float32[3] gyro
float32[3] raw_mag
float32[3] raw_accel
float32[3] raw_gyro"""
  __slots__ = ['header','data']
  _slot_types = ['std_msgs/Header','io_mocap/mocap_data[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(squashed_mocap_data, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.data is None:
        self.data = []
    else:
      self.header = std_msgs.msg.Header()
      self.data = []

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
      length = len(self.data)
      buff.write(_struct_I.pack(length))
      for val1 in self.data:
        _v1 = val1.header
        _x = _v1.seq
        buff.write(_get_struct_I().pack(_x))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        buff.write(_get_struct_4f().pack(*val1.joint_quaternion))
        buff.write(_get_struct_4f().pack(*val1.raw_quaternion))
        buff.write(_get_struct_4f().pack(*val1.world_quaternion))
        buff.write(_get_struct_3f().pack(*val1.mag))
        buff.write(_get_struct_3f().pack(*val1.accel))
        buff.write(_get_struct_3f().pack(*val1.gyro))
        buff.write(_get_struct_3f().pack(*val1.raw_mag))
        buff.write(_get_struct_3f().pack(*val1.raw_accel))
        buff.write(_get_struct_3f().pack(*val1.raw_gyro))
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
      if self.data is None:
        self.data = None
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.data = []
      for i in range(0, length):
        val1 = io_mocap.msg.mocap_data()
        _v3 = val1.header
        start = end
        end += 4
        (_v3.seq,) = _get_struct_I().unpack(str[start:end])
        _v4 = _v3.stamp
        _x = _v4
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v3.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v3.frame_id = str[start:end]
        start = end
        end += 16
        val1.joint_quaternion = _get_struct_4f().unpack(str[start:end])
        start = end
        end += 16
        val1.raw_quaternion = _get_struct_4f().unpack(str[start:end])
        start = end
        end += 16
        val1.world_quaternion = _get_struct_4f().unpack(str[start:end])
        start = end
        end += 12
        val1.mag = _get_struct_3f().unpack(str[start:end])
        start = end
        end += 12
        val1.accel = _get_struct_3f().unpack(str[start:end])
        start = end
        end += 12
        val1.gyro = _get_struct_3f().unpack(str[start:end])
        start = end
        end += 12
        val1.raw_mag = _get_struct_3f().unpack(str[start:end])
        start = end
        end += 12
        val1.raw_accel = _get_struct_3f().unpack(str[start:end])
        start = end
        end += 12
        val1.raw_gyro = _get_struct_3f().unpack(str[start:end])
        self.data.append(val1)
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
      length = len(self.data)
      buff.write(_struct_I.pack(length))
      for val1 in self.data:
        _v5 = val1.header
        _x = _v5.seq
        buff.write(_get_struct_I().pack(_x))
        _v6 = _v5.stamp
        _x = _v6
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v5.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        buff.write(val1.joint_quaternion.tostring())
        buff.write(val1.raw_quaternion.tostring())
        buff.write(val1.world_quaternion.tostring())
        buff.write(val1.mag.tostring())
        buff.write(val1.accel.tostring())
        buff.write(val1.gyro.tostring())
        buff.write(val1.raw_mag.tostring())
        buff.write(val1.raw_accel.tostring())
        buff.write(val1.raw_gyro.tostring())
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
      if self.data is None:
        self.data = None
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
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.data = []
      for i in range(0, length):
        val1 = io_mocap.msg.mocap_data()
        _v7 = val1.header
        start = end
        end += 4
        (_v7.seq,) = _get_struct_I().unpack(str[start:end])
        _v8 = _v7.stamp
        _x = _v8
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v7.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v7.frame_id = str[start:end]
        start = end
        end += 16
        val1.joint_quaternion = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=4)
        start = end
        end += 16
        val1.raw_quaternion = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=4)
        start = end
        end += 16
        val1.world_quaternion = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=4)
        start = end
        end += 12
        val1.mag = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
        start = end
        end += 12
        val1.accel = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
        start = end
        end += 12
        val1.gyro = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
        start = end
        end += 12
        val1.raw_mag = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
        start = end
        end += 12
        val1.raw_accel = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
        start = end
        end += 12
        val1.raw_gyro = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
        self.data.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
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
