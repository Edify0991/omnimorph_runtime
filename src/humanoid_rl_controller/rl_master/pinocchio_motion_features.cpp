#include "rl_master/pinocchio_motion_features.h"

#include "rl_master/robot_state.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace
{

Eigen::Quaterniond normalizedQuatFromXyzw(
    const float x,
    const float y,
    const float z,
    const float w)
{
    Eigen::Quaterniond quat(
        static_cast<double>(w),
        static_cast<double>(x),
        static_cast<double>(y),
        static_cast<double>(z));
    if (!std::isfinite(quat.x()) || !std::isfinite(quat.y()) ||
        !std::isfinite(quat.z()) || !std::isfinite(quat.w()) ||
        quat.norm() < 1.0e-8)
    {
        return Eigen::Quaterniond::Identity();
    }
    quat.normalize();
    return quat;
}

Eigen::Quaterniond yawQuaternion(const Eigen::Quaterniond &quat_xyzw)
{
    const double yaw = std::atan2(
        2.0 * (quat_xyzw.w() * quat_xyzw.z() + quat_xyzw.x() * quat_xyzw.y()),
        1.0 - 2.0 * (quat_xyzw.y() * quat_xyzw.y() + quat_xyzw.z() * quat_xyzw.z()));
    const double half_yaw = 0.5 * yaw;
    Eigen::Quaterniond yaw_only(std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw));
    yaw_only.normalize();
    return yaw_only;
}

std::vector<float> rotationMatrixToRot6(const Eigen::Matrix3d &rotation)
{
    return {
        static_cast<float>(rotation(0, 0)),
        static_cast<float>(rotation(0, 1)),
        static_cast<float>(rotation(1, 0)),
        static_cast<float>(rotation(1, 1)),
        static_cast<float>(rotation(2, 0)),
        static_cast<float>(rotation(2, 1)),
    };
}

Eigen::Vector3d vector3FromSlice(const std::vector<float> &values, const size_t index)
{
    const size_t offset = index * 3;
    return Eigen::Vector3d(
        static_cast<double>(values[offset + 0]),
        static_cast<double>(values[offset + 1]),
        static_cast<double>(values[offset + 2]));
}

Eigen::Quaterniond quaternionFromSliceXyzw(const std::vector<float> &values, const size_t index)
{
    const size_t offset = index * 4;
    return normalizedQuatFromXyzw(
        values[offset + 0],
        values[offset + 1],
        values[offset + 2],
        values[offset + 3]);
}

} // namespace

namespace rl_master
{

PinocchioMotionFeatures::PinocchioMotionFeatures(
    const std::string &urdf_path,
    const std::vector<std::string> &installed_joint_names)
    : data_(model_)
{
    available_ = loadModel(urdf_path, installed_joint_names);
}

bool PinocchioMotionFeatures::available() const
{
    return available_;
}

const std::string &PinocchioMotionFeatures::lastError() const
{
    return last_error_;
}

void PinocchioMotionFeatures::resetAlignment()
{
    alignment_initialized_ = false;
    aligned_anchor_body_.clear();
    world_to_init_ = pinocchio::SE3::Identity();
}

bool PinocchioMotionFeatures::buildMotionLocalFeatures(
    const RobotState &robot,
    const std::vector<std::string> &body_names,
    const std::string &anchor_body,
    const std::vector<float> &reference_body_pos_w,
    const std::vector<float> &reference_body_quat_w_xyzw,
    std::vector<float> *motion_anchor_pos_b,
    std::vector<float> *motion_anchor_ori_b,
    std::vector<float> *motion_body_pos_b,
    std::vector<float> *motion_body_ori_b)
{
    if (!available_)
    {
        return false;
    }
    if (reference_body_pos_w.size() % 3 != 0 || reference_body_quat_w_xyzw.size() % 4 != 0)
    {
        last_error_ = "reference body output shape is invalid";
        return false;
    }

    const size_t body_count = std::min(reference_body_pos_w.size() / 3, reference_body_quat_w_xyzw.size() / 4);
    if (body_count == 0 || body_count != body_names.size())
    {
        last_error_ = "reference body output count does not match reference_body_names";
        return false;
    }
    if (!updateRobotKinematics(robot))
    {
        return false;
    }

    std::vector<pinocchio::FrameIndex> frame_ids;
    pinocchio::FrameIndex anchor_frame_id = 0;
    size_t anchor_index = 0;
    if (!resolveFrameIds(body_names, &frame_ids) ||
        !resolveAnchorFrameId(anchor_body, &anchor_frame_id, &anchor_index, body_names, frame_ids) ||
        !initializeAlignmentIfNeeded(
            anchor_frame_id,
            anchor_index,
            anchor_body,
            reference_body_pos_w,
            reference_body_quat_w_xyzw))
    {
        return false;
    }

    const pinocchio::SE3 &anchor_pose_real = data_.oMf[anchor_frame_id];

    if (motion_anchor_pos_b)
    {
        motion_anchor_pos_b->clear();
    }
    if (motion_anchor_ori_b)
    {
        motion_anchor_ori_b->clear();
    }
    if (motion_body_pos_b)
    {
        motion_body_pos_b->clear();
        motion_body_pos_b->reserve(body_count * 3);
    }
    if (motion_body_ori_b)
    {
        motion_body_ori_b->clear();
        motion_body_ori_b->reserve(body_count * 6);
    }

    for (size_t i = 0; i < body_count; ++i)
    {
        const Eigen::Vector3d ref_pos_world = vector3FromSlice(reference_body_pos_w, i);
        const Eigen::Quaterniond ref_quat_world = quaternionFromSliceXyzw(reference_body_quat_w_xyzw, i);
        const pinocchio::SE3 ref_pose_world(ref_quat_world.toRotationMatrix(), ref_pos_world);
        const pinocchio::SE3 ref_pose_local = anchor_pose_real.actInv(world_to_init_.act(ref_pose_world));

        if (i == anchor_index)
        {
            if (motion_anchor_pos_b)
            {
                motion_anchor_pos_b->assign({
                    static_cast<float>(ref_pose_local.translation().x()),
                    static_cast<float>(ref_pose_local.translation().y()),
                    static_cast<float>(ref_pose_local.translation().z())});
            }
            if (motion_anchor_ori_b)
            {
                *motion_anchor_ori_b = rotationMatrixToRot6(ref_pose_local.rotation());
            }
        }

        if (motion_body_pos_b)
        {
            motion_body_pos_b->push_back(static_cast<float>(ref_pose_local.translation().x()));
            motion_body_pos_b->push_back(static_cast<float>(ref_pose_local.translation().y()));
            motion_body_pos_b->push_back(static_cast<float>(ref_pose_local.translation().z()));
        }
        if (motion_body_ori_b)
        {
            const std::vector<float> rot6 = rotationMatrixToRot6(ref_pose_local.rotation());
            motion_body_ori_b->insert(motion_body_ori_b->end(), rot6.begin(), rot6.end());
        }
    }

    return true;
}

bool PinocchioMotionFeatures::buildRobotBodyLocalFeatures(
    const RobotState &robot,
    const std::vector<std::string> &body_names,
    const std::string &anchor_body,
    std::vector<float> *robot_body_pos_b,
    std::vector<float> *robot_body_ori_b)
{
    if (!available_)
    {
        return false;
    }
    if (!updateRobotKinematics(robot))
    {
        return false;
    }

    std::vector<pinocchio::FrameIndex> frame_ids;
    pinocchio::FrameIndex anchor_frame_id = 0;
    size_t anchor_index = 0;
    if (!resolveFrameIds(body_names, &frame_ids) ||
        !resolveAnchorFrameId(anchor_body, &anchor_frame_id, &anchor_index, body_names, frame_ids))
    {
        return false;
    }
    (void)anchor_index;

    const pinocchio::SE3 &anchor_pose_real = data_.oMf[anchor_frame_id];

    if (robot_body_pos_b)
    {
        robot_body_pos_b->clear();
        robot_body_pos_b->reserve(frame_ids.size() * 3);
    }
    if (robot_body_ori_b)
    {
        robot_body_ori_b->clear();
        robot_body_ori_b->reserve(frame_ids.size() * 6);
    }

    for (const pinocchio::FrameIndex frame_id : frame_ids)
    {
        const pinocchio::SE3 body_pose_local = anchor_pose_real.actInv(data_.oMf[frame_id]);
        if (robot_body_pos_b)
        {
            robot_body_pos_b->push_back(static_cast<float>(body_pose_local.translation().x()));
            robot_body_pos_b->push_back(static_cast<float>(body_pose_local.translation().y()));
            robot_body_pos_b->push_back(static_cast<float>(body_pose_local.translation().z()));
        }
        if (robot_body_ori_b)
        {
            const std::vector<float> rot6 = rotationMatrixToRot6(body_pose_local.rotation());
            robot_body_ori_b->insert(robot_body_ori_b->end(), rot6.begin(), rot6.end());
        }
    }

    return true;
}

bool PinocchioMotionFeatures::loadModel(
    const std::string &urdf_path,
    const std::vector<std::string> &installed_joint_names)
{
    try
    {
        pinocchio::urdf::buildModel(
            urdf_path,
            pinocchio::JointModelFreeFlyer(),
            model_);
        data_ = pinocchio::Data(model_);
        q_buffer_ = pinocchio::neutral(model_);
        frame_id_cache_.clear();
        joint_q_indices_.clear();
        joint_q_indices_.reserve(installed_joint_names.size());

        for (const std::string &joint_name : installed_joint_names)
        {
            if (!model_.existJointName(joint_name))
            {
                last_error_ = "Pinocchio model is missing joint '" + joint_name + "'";
                return false;
            }
            const pinocchio::JointIndex joint_id = model_.getJointId(joint_name);
            const auto &joint_model = model_.joints[joint_id];
            if (joint_model.nq() != 1)
            {
                last_error_ = "Pinocchio joint '" + joint_name + "' is not 1-DOF";
                return false;
            }
            joint_q_indices_.push_back(joint_model.idx_q());
        }
        resetAlignment();
        return true;
    }
    catch (const std::exception &e)
    {
        last_error_ = e.what();
        return false;
    }
}

bool PinocchioMotionFeatures::updateRobotKinematics(const RobotState &robot)
{
    if (robot.joint_q.size() < joint_q_indices_.size())
    {
        last_error_ = "robot joint_q size is smaller than installed joint mapping";
        return false;
    }
    if (robot.base_pos_w.size() < 3 || robot.base_quat.size() < 4)
    {
        last_error_ = "robot base pose is incomplete";
        return false;
    }

    q_buffer_ = pinocchio::neutral(model_);
    q_buffer_[0] = static_cast<double>(robot.base_pos_w[0]);
    q_buffer_[1] = static_cast<double>(robot.base_pos_w[1]);
    q_buffer_[2] = static_cast<double>(robot.base_pos_w[2]);

    const Eigen::Quaterniond base_quat = normalizedQuatFromXyzw(
        robot.base_quat[0],
        robot.base_quat[1],
        robot.base_quat[2],
        robot.base_quat[3]);
    q_buffer_[3] = base_quat.x();
    q_buffer_[4] = base_quat.y();
    q_buffer_[5] = base_quat.z();
    q_buffer_[6] = base_quat.w();

    for (size_t i = 0; i < joint_q_indices_.size(); ++i)
    {
        q_buffer_[joint_q_indices_[i]] = static_cast<double>(robot.joint_q[i]);
    }

    pinocchio::forwardKinematics(model_, data_, q_buffer_);
    pinocchio::updateFramePlacements(model_, data_);
    return true;
}

bool PinocchioMotionFeatures::resolveFrameIds(
    const std::vector<std::string> &body_names,
    std::vector<pinocchio::FrameIndex> *frame_ids)
{
    if (!frame_ids)
    {
        last_error_ = "frame_ids output is null";
        return false;
    }
    frame_ids->clear();
    frame_ids->reserve(body_names.size());
    for (const std::string &body_name : body_names)
    {
        pinocchio::FrameIndex frame_id = 0;
        if (!resolveFrameIdByName(body_name, &frame_id))
        {
            return false;
        }
        frame_ids->push_back(frame_id);
    }
    return true;
}

bool PinocchioMotionFeatures::resolveAnchorFrameId(
    const std::string &anchor_body,
    pinocchio::FrameIndex *anchor_frame_id,
    size_t *anchor_index,
    const std::vector<std::string> &body_names,
    const std::vector<pinocchio::FrameIndex> &frame_ids)
{
    if (!anchor_frame_id || !anchor_index)
    {
        last_error_ = "anchor outputs are null";
        return false;
    }
    const auto it = std::find(body_names.begin(), body_names.end(), anchor_body);
    if (it == body_names.end())
    {
        last_error_ = "anchor body '" + anchor_body + "' is not present in body_names";
        return false;
    }
    *anchor_index = static_cast<size_t>(std::distance(body_names.begin(), it));
    if (*anchor_index >= frame_ids.size())
    {
        last_error_ = "anchor index is out of frame range";
        return false;
    }
    *anchor_frame_id = frame_ids[*anchor_index];
    return true;
}

bool PinocchioMotionFeatures::initializeAlignmentIfNeeded(
    pinocchio::FrameIndex anchor_frame_id,
    size_t anchor_index,
    const std::string &anchor_body,
    const std::vector<float> &reference_body_pos_w,
    const std::vector<float> &reference_body_quat_w_xyzw)
{
    if (alignment_initialized_ && aligned_anchor_body_ == anchor_body)
    {
        return true;
    }

    const Eigen::Vector3d anchor_ref_pos = vector3FromSlice(reference_body_pos_w, anchor_index);
    const Eigen::Quaterniond anchor_ref_quat = quaternionFromSliceXyzw(reference_body_quat_w_xyzw, anchor_index);

    pinocchio::SE3 init_to_anchor(anchor_ref_quat.toRotationMatrix(), anchor_ref_pos);
    pinocchio::SE3 world_to_anchor = data_.oMf[anchor_frame_id];

    init_to_anchor.rotation() = yawQuaternion(Eigen::Quaterniond(init_to_anchor.rotation())).toRotationMatrix();
    world_to_anchor.rotation() = yawQuaternion(Eigen::Quaterniond(world_to_anchor.rotation())).toRotationMatrix();

    world_to_init_ = world_to_anchor * init_to_anchor.inverse();
    alignment_initialized_ = true;
    aligned_anchor_body_ = anchor_body;
    return true;
}

bool PinocchioMotionFeatures::resolveFrameIdByName(
    const std::string &name,
    pinocchio::FrameIndex *frame_id)
{
    if (!frame_id)
    {
        last_error_ = "frame_id output is null";
        return false;
    }

    const auto cache_it = frame_id_cache_.find(name);
    if (cache_it != frame_id_cache_.end())
    {
        *frame_id = cache_it->second;
        return true;
    }

    const pinocchio::FrameType joint_like_types =
        static_cast<pinocchio::FrameType>(pinocchio::JOINT | pinocchio::FIXED_JOINT);
    if (model_.existFrame(name, pinocchio::BODY))
    {
        *frame_id = model_.getFrameId(name, pinocchio::BODY);
    }
    else if (model_.existFrame(name, joint_like_types))
    {
        *frame_id = model_.getFrameId(name, joint_like_types);
    }
    else if (model_.existFrame(name, pinocchio::OP_FRAME))
    {
        *frame_id = model_.getFrameId(name, pinocchio::OP_FRAME);
    }
    else
    {
        last_error_ = "Pinocchio model is missing frame '" + name + "'";
        return false;
    }

    frame_id_cache_[name] = *frame_id;
    return true;
}

} // namespace rl_master
