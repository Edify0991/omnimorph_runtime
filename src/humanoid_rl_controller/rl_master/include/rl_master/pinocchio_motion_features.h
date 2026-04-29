#ifndef RL_MASTER_PINOCCHIO_MOTION_FEATURES_H
#define RL_MASTER_PINOCCHIO_MOTION_FEATURES_H

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <string>
#include <unordered_map>
#include <vector>

class RobotState;

namespace rl_master
{

class PinocchioMotionFeatures
{
public:
    explicit PinocchioMotionFeatures(
        const std::string &urdf_path,
        const std::vector<std::string> &installed_joint_names);

    bool available() const;
    const std::string &lastError() const;
    void resetAlignment();

    bool buildMotionLocalFeatures(
        const RobotState &robot,
        const std::vector<std::string> &body_names,
        const std::string &anchor_body,
        const std::vector<float> &reference_body_pos_w,
        const std::vector<float> &reference_body_quat_w_xyzw,
        std::vector<float> *motion_anchor_pos_b,
        std::vector<float> *motion_anchor_ori_b,
        std::vector<float> *motion_body_pos_b,
        std::vector<float> *motion_body_ori_b);

    bool buildRobotBodyLocalFeatures(
        const RobotState &robot,
        const std::vector<std::string> &body_names,
        const std::string &anchor_body,
        std::vector<float> *robot_body_pos_b,
        std::vector<float> *robot_body_ori_b);

private:
    bool loadModel(const std::string &urdf_path, const std::vector<std::string> &installed_joint_names);
    bool updateRobotKinematics(const RobotState &robot);
    bool resolveFrameIds(
        const std::vector<std::string> &body_names,
        std::vector<pinocchio::FrameIndex> *frame_ids);
    bool resolveAnchorFrameId(
        const std::string &anchor_body,
        pinocchio::FrameIndex *anchor_frame_id,
        size_t *anchor_index,
        const std::vector<std::string> &body_names,
        const std::vector<pinocchio::FrameIndex> &frame_ids);
    bool initializeAlignmentIfNeeded(
        pinocchio::FrameIndex anchor_frame_id,
        size_t anchor_index,
        const std::string &anchor_body,
        const std::vector<float> &reference_body_pos_w,
        const std::vector<float> &reference_body_quat_w_xyzw);
    bool resolveFrameIdByName(const std::string &name, pinocchio::FrameIndex *frame_id);

    pinocchio::Model model_;
    pinocchio::Data data_;
    std::vector<int> joint_q_indices_;
    pinocchio::Model::ConfigVectorType q_buffer_;
    std::unordered_map<std::string, pinocchio::FrameIndex> frame_id_cache_;
    pinocchio::SE3 world_to_init_;
    bool available_ = false;
    bool alignment_initialized_ = false;
    std::string aligned_anchor_body_;
    std::string last_error_;
};

} // namespace rl_master

#endif // RL_MASTER_PINOCCHIO_MOTION_FEATURES_H
