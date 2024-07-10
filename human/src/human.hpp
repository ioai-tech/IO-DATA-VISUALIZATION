#ifndef RVIZ_FOOT_VISUALIZER_SRC_HUMAN_SRC_HUMAN_HPP_
#define RVIZ_FOOT_VISUALIZER_SRC_HUMAN_SRC_HUMAN_HPP_

#include <geometry_msgs/TransformStamped.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "io_msgs/mocap_data.h"
#include "io_msgs/squashed_mocap_data.h"
#include "ros/node_handle.h"
#include "visualization_msgs/MarkerArray.h"

static const std::vector<std::string> kJointNames = {
    "joint_Hips_R",
    "joint_Hips_P",
    "joint_Hips_Y",
    "joint_RightUpLeg_R",
    "joint_RightUpLeg_P",
    "joint_RightUpLeg_Y",
    "joint_RightLeg_R",
    "joint_RightLeg_P",
    "joint_RightLeg_Y",
    "joint_RightFoot_R",
    "joint_RightFoot_P",
    "joint_RightFoot_Y",
    "joint_LeftUpLeg_R",
    "joint_LeftUpLeg_P",
    "joint_LeftUpLeg_Y",
    "joint_LeftLeg_R",
    "joint_LeftLeg_P",
    "joint_LeftLeg_Y",
    "joint_LeftFoot_R",
    "joint_LeftFoot_P",
    "joint_LeftFoot_Y",
    "joint_Spine_R",
    "joint_Spine_P",
    "joint_Spine_Y",
    "joint_Spine1_R",
    "joint_Spine1_P",
    "joint_Spine1_Y",
    "joint_Spine2_R",
    "joint_Spine2_P",
    "joint_Spine2_Y",
    "joint_Neck_R",
    "joint_Neck_P",
    "joint_Neck_Y",
    "joint_Neck1_R",
    "joint_Neck1_P",
    "joint_Neck1_Y",
    "joint_Head_R",
    "joint_Head_P",
    "joint_Head_Y",
    "joint_RightShoulder_R",
    "joint_RightShoulder_P",
    "joint_RightShoulder_Y",
    "joint_RightArm_R",
    "joint_RightArm_P",
    "joint_RightArm_Y",
    "joint_RightForeArm_R",
    "joint_RightForeArm_P",
    "joint_RightForeArm_Y",
    "joint_RightHand_R",
    "joint_RightHand_P",
    "joint_RightHand_Y",
    "joint_RightHandThumb1_R",
    "joint_RightHandThumb1_P",
    "joint_RightHandThumb1_Y",
    "joint_RightHandThumb2_R",
    "joint_RightHandThumb2_P",
    "joint_RightHandThumb2_Y",
    "joint_RightHandThumb3_R",
    "joint_RightHandThumb3_P",
    "joint_RightHandThumb3_Y",
    "joint_RightInHandIndex_R",
    "joint_RightInHandIndex_P",
    "joint_RightInHandIndex_Y",
    "joint_RightHandIndex1_R",
    "joint_RightHandIndex1_P",
    "joint_RightHandIndex1_Y",
    "joint_RightHandIndex2_R",
    "joint_RightHandIndex2_P",
    "joint_RightHandIndex2_Y",
    "joint_RightHandIndex3_R",
    "joint_RightHandIndex3_P",
    "joint_RightHandIndex3_Y",
    "joint_RightInHandMiddle_R",
    "joint_RightInHandMiddle_P",
    "joint_RightInHandMiddle_Y",
    "joint_RightHandMiddle1_R",
    "joint_RightHandMiddle1_P",
    "joint_RightHandMiddle1_Y",
    "joint_RightHandMiddle2_R",
    "joint_RightHandMiddle2_P",
    "joint_RightHandMiddle2_Y",
    "joint_RightHandMiddle3_R",
    "joint_RightHandMiddle3_P",
    "joint_RightHandMiddle3_Y",
    "joint_RightInHandRing_R",
    "joint_RightInHandRing_P",
    "joint_RightInHandRing_Y",
    "joint_RightHandRing1_R",
    "joint_RightHandRing1_P",
    "joint_RightHandRing1_Y",
    "joint_RightHandRing2_R",
    "joint_RightHandRing2_P",
    "joint_RightHandRing2_Y",
    "joint_RightHandRing3_R",
    "joint_RightHandRing3_P",
    "joint_RightHandRing3_Y",
    "joint_RightInHandPinky_R",
    "joint_RightInHandPinky_P",
    "joint_RightInHandPinky_Y",
    "joint_RightHandPinky1_R",
    "joint_RightHandPinky1_P",
    "joint_RightHandPinky1_Y",
    "joint_RightHandPinky2_R",
    "joint_RightHandPinky2_P",
    "joint_RightHandPinky2_Y",
    "joint_RightHandPinky3_R",
    "joint_RightHandPinky3_P",
    "joint_RightHandPinky3_Y",
    "joint_LeftShoulder_R",
    "joint_LeftShoulder_P",
    "joint_LeftShoulder_Y",
    "joint_LeftArm_R",
    "joint_LeftArm_P",
    "joint_LeftArm_Y",
    "joint_LeftForeArm_R",
    "joint_LeftForeArm_P",
    "joint_LeftForeArm_Y",
    "joint_LeftHand_R",
    "joint_LeftHand_P",
    "joint_LeftHand_Y",
    "joint_LeftHandThumb1_R",
    "joint_LeftHandThumb1_P",
    "joint_LeftHandThumb1_Y",
    "joint_LeftHandThumb2_R",
    "joint_LeftHandThumb2_P",
    "joint_LeftHandThumb2_Y",
    "joint_LeftHandThumb3_R",
    "joint_LeftHandThumb3_P",
    "joint_LeftHandThumb3_Y",
    "joint_LeftInHandIndex_R",
    "joint_LeftInHandIndex_P",
    "joint_LeftInHandIndex_Y",
    "joint_LeftHandIndex1_R",
    "joint_LeftHandIndex1_P",
    "joint_LeftHandIndex1_Y",
    "joint_LeftHandIndex2_R",
    "joint_LeftHandIndex2_P",
    "joint_LeftHandIndex2_Y",
    "joint_LeftHandIndex3_R",
    "joint_LeftHandIndex3_P",
    "joint_LeftHandIndex3_Y",
    "joint_LeftInHandMiddle_R",
    "joint_LeftInHandMiddle_P",
    "joint_LeftInHandMiddle_Y",
    "joint_LeftHandMiddle1_R",
    "joint_LeftHandMiddle1_P",
    "joint_LeftHandMiddle1_Y",
    "joint_LeftHandMiddle2_R",
    "joint_LeftHandMiddle2_P",
    "joint_LeftHandMiddle2_Y",
    "joint_LeftHandMiddle3_R",
    "joint_LeftHandMiddle3_P",
    "joint_LeftHandMiddle3_Y",
    "joint_LeftInHandRing_R",
    "joint_LeftInHandRing_P",
    "joint_LeftInHandRing_Y",
    "joint_LeftHandRing1_R",
    "joint_LeftHandRing1_P",
    "joint_LeftHandRing1_Y",
    "joint_LeftHandRing2_R",
    "joint_LeftHandRing2_P",
    "joint_LeftHandRing2_Y",
    "joint_LeftHandRing3_R",
    "joint_LeftHandRing3_P",
    "joint_LeftHandRing3_Y",
    "joint_LeftInHandPinky_R",
    "joint_LeftInHandPinky_P",
    "joint_LeftInHandPinky_Y",
    "joint_LeftHandPinky1_R",
    "joint_LeftHandPinky1_P",
    "joint_LeftHandPinky1_Y",
    "joint_LeftHandPinky2_R",
    "joint_LeftHandPinky2_P",
    "joint_LeftHandPinky2_Y",
    "joint_LeftHandPinky3_R",
    "joint_LeftHandPinky3_P",
    "joint_LeftHandPinky3_Y",
    "joint_Head_Camera",
};

class Human {
public:
  Human() {
    model_.initParam("robot_description");
    frameid_to_index_ = {
        {"base", 0},        {"r-thigh", 1},     {"r-leg", 2},
        {"r-foot", 3},      {"l-thigh", 4},     {"l-leg", 5},
        {"l-foot", 6},      {"dorsal", 9},      {"head", 12},
        {"r-shoulder", 13},
        {"r-arm", 14},      {"r-forarm", 15},
        {"r-hand", 16},     {"r-thumb-0", 18},  {"r-thumb-1", 19},
        {"r-index-0", 21},  {"r-index-1", 22},  {"r-middle-0", 25},
        {"r-middle-1", 26}, {"r-ring-0", 29},   {"r-ring-1", 30},
        {"r-pinky-0", 33},  {"r-pinky-1", 34},
        {"l-shoulder", 36},
        {"l-arm", 37},      {"l-forarm", 38},   {"l-hand", 39},
        {"l-thumb-0", 41},  {"l-thumb-1", 42},  {"l-index-0", 44},
        {"l-index-1", 45},  {"l-middle-0", 48}, {"l-middle-1", 49},
        {"l-ring-0", 52},   {"l-ring-1", 53},   {"l-pinky-0", 56},
        {"l-pinky-1", 57},
    };
    cnt_ = 0;
    InitSubPub();
  }
  void ExtractSquashedData(const io_msgs::squashed_mocap_data::ConstPtr &msg) {
    // joint_state_pub_.publish(GetJointState(msg));
    for (auto &it : msg->data) {
      for (int i = 0; i < 4; i++) {
        std::array<float, 4> ori = {it.joint_quaternion[0], it.joint_quaternion[1],
                                    it.joint_quaternion[2], it.joint_quaternion[3]};
        frameid_to_quaternion_[it.header.frame_id] = ori;
      }
    }
  }
  void ExtractUnsquashedData(const io_msgs::mocap_data::ConstPtr &msg) {
    for (int i = 0; i < 4; i++) {
      std::array<float, 4> ori = {msg->joint_quaternion[0], msg->joint_quaternion[1],
                                  msg->joint_quaternion[2], msg->joint_quaternion[3]};
      frameid_to_quaternion_[msg->header.frame_id] = ori;
    }
  }
  inline void InitSubPub() {
    mocap_sub_ = n_.subscribe<io_msgs::squashed_mocap_data>(
        "/mocap_data", 10,
        [this](const io_msgs::squashed_mocap_data::ConstPtr &msg) {
          this->ExtractSquashedData(msg);
        });
    for (auto const &p : frameid_to_index_) {
      std::string frame_id = p.first;
      std::replace(frame_id.begin(), frame_id.end(), '-', '_');
      std::string topic_name = "/mocap_" + frame_id;
      std::cerr << topic_name << std::endl;
      unsquashed_subscribers_.emplace_back(n_.subscribe<io_msgs::mocap_data>(
          topic_name, 10, [this](const io_msgs::mocap_data::ConstPtr &msg) {
            this->ExtractUnsquashedData(msg);
          }));
    }
    joint_state_pub_ =
        n_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  }
  struct Quaternion {
    double w, x, y, z;
  };
  struct EulerAngle {
    double roll, pitch, yaw;
  };
  struct Matrix {
    double m[3][3];
  };

  Matrix QuaternionToRotationMatrix(Quaternion q) {
    Matrix matrix;

    double sqw = q.w * q.w;
    double sqx = q.x * q.x;
    double sqy = q.y * q.y;
    double sqz = q.z * q.z;

    // 计算旋转矩阵元素
    matrix.m[0][0] = sqx - sqy - sqz + sqw;
    matrix.m[1][1] = -sqx + sqy - sqz + sqw;
    matrix.m[2][2] = -sqx - sqy + sqz + sqw;

    double tmp1 = q.x * q.y;
    double tmp2 = q.z * q.w;
    matrix.m[1][0] = 2.0 * (tmp1 + tmp2);
    matrix.m[0][1] = 2.0 * (tmp1 - tmp2);

    tmp1 = q.x * q.z;
    tmp2 = q.y * q.w;
    matrix.m[2][0] = 2.0 * (tmp1 - tmp2);
    matrix.m[0][2] = 2.0 * (tmp1 + tmp2);

    tmp1 = q.y * q.z;
    tmp2 = q.x * q.w;
    matrix.m[2][1] = 2.0 * (tmp1 + tmp2);
    matrix.m[1][2] = 2.0 * (tmp1 - tmp2);

    return matrix;
  }
  EulerAngle RotationMatrixToEuler(Matrix rotation) {
    double r11, r12, r13, r21, r22, r23, r33;
    r11 = rotation.m[0][0];
    r12 = rotation.m[0][1];
    r13 = rotation.m[0][2];
    r21 = rotation.m[1][0];
    r22 = rotation.m[1][1];
    r23 = rotation.m[1][2];
    r33 = rotation.m[2][2];
    double alpha, belta, gama;
    double ZERO = 1e-5;
    if (fabs(r23) < ZERO && fabs(r33) < ZERO) {
      alpha = 0;
      belta = M_PI_2;
      gama = atan2(r21, r22);
    } else {
      alpha = atan2(-r23, r33);
      belta = atan2(r13, sqrt(r11 * r11 + r12 * r12));
      gama = atan2(-r12, r11);
    }
    return {alpha, belta, gama};
  }

  Quaternion Slerp(Quaternion q1, Quaternion q2, double t) {
    double cos_half_theta =
        q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

    if (cos_half_theta < 0.0) {
      q2.w = -q2.w;
      q2.x = -q2.x;
      q2.y = -q2.y;
      q2.z = -q2.z;
      cos_half_theta = -cos_half_theta;
    }

    double k0, k1;

    if (cos_half_theta > 0.9999) {
      k0 = 1.0 -t;
      k1 = t;
    } else {
      double sin_half_theta = sqrt(1.0 - cos_half_theta * cos_half_theta);
      double omega = atan2(sin_half_theta, cos_half_theta);
      k0 = sin((1.0 - t) * omega) / sin_half_theta;
      k1 = sin(t * omega) / sin_half_theta;
    }

    double w, x, y, z;

    Quaternion result;

    if (cos_half_theta >= 0) {
      result.w = k0 * q1.w + k1 * q2.w;
      result.x = k0 * q1.x + k1 * q2.x;
      result.y = k0 * q1.y + k1 * q2.y;
      result.z = k0 * q1.z + k1 * q2.z;
    } else {
      result.w = k0 * q1.w - k1 * q2.w;
      result.x = k0 * q1.x - k1 * q2.x;
      result.y = k0 * q1.y - k1 * q2.y;
      result.z = k0 * q1.z - k1 * q2.z;
    }

    // if (fabs(cos_half_theta) >= 1.0) {
    //   return q1;
    // }

    // double half_theta = acos(cos_half_theta);
    // double sin_half_theta = sqrt(1.0 - cos_half_theta * cos_half_theta);

    // // Check if sin_half_theta is very close to zero
    // if (fabs(sin_half_theta) < 0.0001) {
    //   // If it is, perform a linear interpolation instead
    //   Quaternion result;
    //   result.w = q1.w * (1 - t) + q2.w * t;
    //   result.x = q1.x * (1 - t) + q2.x * t;
    //   result.y = q1.y * (1 - t) + q2.y * t;
    //   result.z = q1.z * (1 - t) + q2.z * t;
    //   return result;
    // }

    // double weight_q1 = sin((1 - t) * half_theta) / sin_half_theta;
    // double weight_q2 = sin(t * half_theta) / sin_half_theta;


    // result.w = q1.w * weight_q1 + q2.w * weight_q2;
    // result.x = q1.x * weight_q1 + q2.x * weight_q2;
    // result.y = q1.y * weight_q1 + q2.y * weight_q2;
    // result.z = q1.z * weight_q1 + q2.z * weight_q2;

    return result;
  }

  void Publish() {
    sensor_msgs::JointState joint_state;
    joint_state.position.resize(kJointNames.size());
    joint_state.header.frame_id = "base_link";
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.seq = cnt_++;
    joint_state.name = kJointNames;
    auto copy = frameid_to_quaternion_;
    static std::unordered_map<std::string, Quaternion> last_quat;

    for (auto &it : copy) {
      // 手部第一关节单自由度
      if (it.first.find("-1") != std::string::npos) {
        Quaternion q = {it.second[0], it.second[1], it.second[2], it.second[3]};

        if (last_quat.find(it.first) == last_quat.end()) {
          last_quat[it.first] = q;
        } else {
          if (((last_quat[it.first].w > 0.0 && q.w < 0.0) ||
              (last_quat[it.first].w < 0.0 && q.w > 0.0)) &&
              ((last_quat[it.first].y > 0.0 && -q.y < 0.0) ||
              (last_quat[it.first].y < 0.0 && -q.y > 0.0))) {
              q = last_quat[it.first];
          } else {
              last_quat[it.first] = q;
          }
        }

        Quaternion q0 = {1, 0, 0, 0};

        Quaternion q1 = Slerp(q0, q, 0.6);
        Quaternion q2 = Slerp(q0, q, 0.4);
        Matrix m1 = QuaternionToRotationMatrix(q1);
        EulerAngle euler1 = RotationMatrixToEuler(m1);
        joint_state.position[frameid_to_index_[it.first] * 3 + 1] =
            euler1.pitch;

        Matrix m_test = QuaternionToRotationMatrix(q);
        EulerAngle euler_test = RotationMatrixToEuler(m_test);
        // std::cout << "q1: " << it.first << " " << euler1.roll / M_PI * 180 << " " << euler1.pitch / M_PI * 180 << " " << euler1.yaw / M_PI * 180 << std::endl;
        // std::cout << "q1 quat: " << it.first << " " << q1.w << " " << q1.x << " " << q1.y << " " << q1.z << std::endl;
        // std::cout << "q: " << it.first << " " << euler_test.roll / M_PI * 180 << " " << euler_test.pitch / M_PI * 180 << " " << euler_test.yaw / M_PI * 180 << std::endl;
        // std::cout << "q quat: " << it.first << " " << q.w << " " << q.x << " " << q.y << " " << q.z << std::endl;

        if (it.first.find("thumb") == std::string::npos) {
          Matrix m2 = QuaternionToRotationMatrix(q2);
          EulerAngle euler2 = RotationMatrixToEuler(m2);
          joint_state.position[(frameid_to_index_[it.first] + 1) * 3 + 1] =
              euler2.pitch;
        }
      } else {
        Quaternion q = {it.second[0], it.second[1], it.second[2], it.second[3]};
        Matrix m = QuaternionToRotationMatrix(q);
        EulerAngle euler = RotationMatrixToEuler(m);
        joint_state.position[frameid_to_index_[it.first] * 3 + 0] = euler.roll;
        joint_state.position[frameid_to_index_[it.first] * 3 + 1] = euler.pitch;
        joint_state.position[frameid_to_index_[it.first] * 3 + 2] = euler.yaw;
      }
    }
    joint_state_pub_.publish(joint_state);
  }

 private:
  int cnt_;
  ros::NodeHandle n_;
  urdf::Model model_;
  ros::Publisher joint_state_pub_;
  ros::Subscriber mocap_sub_;  //  Subscribe Squashed Data
  std::vector<ros::Subscriber> unsquashed_subscribers_;
  std::map<std::string, int> frameid_to_index_;
  std::unordered_map<std::string, std::array<float, 4>> frameid_to_quaternion_;
};

#endif //  RVIZ_FOOT_VISUALIZER_SRC_HUMAN_SRC_HUMAN_HPP_
