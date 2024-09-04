import sys
import os
import time
import json
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from io_mocap.msg import mocap_data
from io_mocap.msg import touch

IMAGE_FOLDER = {
    "cam_rgb": "images/cam_rgb",
    # "cam_depth": "images/cam_depth",
    "cam_left": "images/cam_left",
    "cam_right": "images/cam_right",
    "cam_fisheye": "images/cam_fisheye",
}

MOCAP_FRAME_NAMES = [
    "base",
    "dorsal",
    "head",
    "l_arm",
    "l_foot",
    "l_forarm",
    "l_hand",
    "l_index_0",
    "l_index_1",
    "l_leg",
    "l_middle_0",
    "l_middle_1",
    "l_pinky_0",
    "l_pinky_1",
    "l_ring_0",
    "l_ring_1",
    "l_thigh",
    "l_thumb_0",
    "l_thumb_1",
    "r_arm",
    "r_foot",
    "r_forarm",
    "r_hand",
    "r_index_0",
    "r_index_1",
    "r_leg",
    "r_middle_0",
    "r_middle_1",
    "r_pinky_0",
    "r_pinky_1",
    "r_ring_0",
    "r_ring_1",
    "r_thigh",
    "r_thumb_0",
    "r_thumb_1",
]

HAPTIC_FRAME_NAMES = ["left_hand", "right_hand", "left_foot", "right_foot"]

if __name__ == "__main__":
    rospy.init_node("rawdata_to_ros1")
    input_folder = sys.argv[1]
    pub_mocap = True  # use mocap data or joint_states data to vis human movement
    if len(sys.argv) > 2:
        pub_mocap = sys.argv[2] == "True"
    image_frame_names = []
    image_publishers = {
        "cam_rgb": rospy.Publisher(
            "/rgbd/color/image_raw/compressed", CompressedImage, queue_size=2
        ),
        "cam_depth": rospy.Publisher("/rgbd/depth/image_raw", Image, queue_size=2),
        "cam_left": rospy.Publisher(
            "/csi_camera_0/image_raw/compressed", CompressedImage, queue_size=2
        ),
        "cam_right": rospy.Publisher(
            "/csi_camera_1/image_raw/compressed", CompressedImage, queue_size=2
        ),
        "cam_fisheye": rospy.Publisher(
            "/usb_cam_fisheye/mjpeg_raw/compressed", CompressedImage, queue_size=2
        ),
    }
    mocap_publishers = {
        name: rospy.Publisher("/mocap_" + name, mocap_data, queue_size=2)
        for name in MOCAP_FRAME_NAMES
    }
    human_joint_states_publisher = rospy.Publisher(
        "/joint_states", JointState, queue_size=2
    )
    haptic_publishers = {
        name: rospy.Publisher("/touch_" + name, touch, queue_size=2)
        for name in HAPTIC_FRAME_NAMES
    }
    task_description_publisher = rospy.Publisher(
        "/task_description", String, queue_size=2
    )
    annotation = json.load(open(os.path.join(input_folder, "annotation.json")))
    start_frame = int(annotation["start_frame_id"])
    end_frame = int(annotation["end_frame_id"])

    frames = {}
    frames["image"] = [{} for _ in range(end_frame - start_frame + 1)]
    for image_type in IMAGE_FOLDER:
        image_names = os.listdir(os.path.join(input_folder, IMAGE_FOLDER[image_type]))
        for index in range(start_frame, end_frame + 1):
            prefix = "%06d" % index
            for name in image_names:
                if name.startswith(prefix):
                    if image_type != "cam_depth":
                        frames["image"][index - start_frame][image_type] = open(
                            os.path.join(input_folder, IMAGE_FOLDER[image_type], name),
                            "rb",
                        ).read(-1)
                    else:
                        frames["image"][index - start_frame][image_type] = cv2.imread(
                            os.path.join(input_folder, IMAGE_FOLDER[image_type], name),
                            cv2.IMREAD_ANYDEPTH,
                        )

    frames["mocap"] = [{} for _ in range(end_frame - start_frame + 1)]
    for mocap_frame in MOCAP_FRAME_NAMES:
        csv_filepath = os.path.join(input_folder, mocap_frame + ".csv")
        if os.path.exists(csv_filepath):
            with open(csv_filepath) as f:
                lines = [x.strip() for x in f.readlines()[1:] if x.strip()]
                for index, line in enumerate(lines):
                    data = line.split(",")
                    frames["mocap"][index][mocap_frame] = [float(x) for x in data]
    frames["joint_states"] = [[] for _ in range(end_frame - start_frame + 1)]
    csv_filepath = os.path.join(input_folder, "joint_states.csv")
    if os.path.exists(csv_filepath):
        with open(csv_filepath) as f:
            lines = [x.strip() for x in f.readlines() if x.strip()]
            for index, line in enumerate(lines):
                if index == 0:
                    human_joint_names = line.split(",")[1:]
                    assert len(human_joint_names) == 177  # 59 shpere joints
                    continue
                data = line.split(",")[1:]
                frames["joint_states"][index - 1] = [float(x) for x in data]
    frames["haptic"] = [{} for _ in range(end_frame - start_frame + 1)]
    for haptic_frame in HAPTIC_FRAME_NAMES:
        csv_filepath = os.path.join(input_folder, "haptics", haptic_frame + ".csv")
        if os.path.exists(csv_filepath):
            with open(csv_filepath) as f:
                lines = [x.strip() for x in f.readlines()[1:] if x.strip()]
                for index, line in enumerate(lines):
                    data = line.split(",")
                    frames["haptic"][index][haptic_frame] = [int(x) for x in data]
                    
    label = annotation["description"]
    task_description_publisher.publish(String(data=str(label)))
    # print("Start publishing images")
    for index in range(end_frame - start_frame + 1):
        if rospy.is_shutdown():
            break
        now = rospy.Time.now()
        # print("publish", index)
        for image_type in IMAGE_FOLDER:
            frame = frames["image"][index]
            if image_type != "cam_depth":
                if image_type in frame and frame[image_type] is not None:
                    image = frame[image_type]
                    msg = CompressedImage()
                    msg.header.stamp = now
                    msg.header.frame_id = image_type
                    msg.header.seq = index
                    msg.format = "jpeg"
                    msg.data = image
                    image_publishers[image_type].publish(msg)
            else:
                if image_type in frame and frame[image_type] is not None:
                    image = frame[image_type]
                    msg = Image()
                    msg.header.stamp = now
                    msg.header.frame_id = image_type
                    msg.header.seq = index
                    msg.encoding = "16UC1"
                    msg.height = image.shape[0]
                    msg.width = image.shape[1]
                    msg.step = image.shape[1] * 2
                    msg.is_bigendian = False
                    msg.data = image.tobytes()
                    image_publishers[image_type].publish(msg)
        if pub_mocap:
            frame = frames["mocap"][index]
            for mocap_frame in MOCAP_FRAME_NAMES:
                if mocap_frame in frame and frame[mocap_frame] is not None:
                    msg = mocap_data()
                    msg.header.stamp = now
                    msg.header.frame_id = mocap_frame.replace("_", "-")
                    msg.header.seq = index
                    if len(frame[mocap_frame]) < 6:
                        continue
                    joint_quat = frame[mocap_frame][2:6]
                    msg.joint_quaternion = joint_quat
                    mocap_publishers[mocap_frame].publish(msg)
        else:
            # print("pub joint_states")
            frame = frames["joint_states"][index]
            msg = JointState()
            msg.header.stamp = now
            msg.header.frame_id = "base_link"
            msg.header.seq = index
            msg.name = human_joint_names
            msg.position = frame
            human_joint_states_publisher.publish(msg)
        frame = frames["haptic"][index]
        for haptic_frame in HAPTIC_FRAME_NAMES:
            if haptic_frame in frame and frame[haptic_frame] is not None:
                msg = touch()
                msg.header.stamp = now
                msg.header.frame_id = haptic_frame
                msg.header.seq = index
                for index, value in enumerate(frame[haptic_frame][2:]):
                    msg.data[index] = value
                haptic_publishers[haptic_frame].publish(msg)
        time.sleep(1.0 / 30.0)
