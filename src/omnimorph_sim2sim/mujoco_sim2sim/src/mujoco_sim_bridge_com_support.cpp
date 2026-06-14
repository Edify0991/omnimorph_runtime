#include "mujoco_sim_bridge_internal.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace mujoco_sim2sim
{
using namespace bridge_internal;

namespace
{
std::array<double, 4> normalizedXyzw(const double x, const double y, const double z, const double w)
{
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (!std::isfinite(norm) || norm < 1.0e-9)
    {
        return {0.0, 0.0, 0.0, 1.0};
    }
    return {x / norm, y / norm, z / norm, w / norm};
}

double cross2d(
    const std::array<double, 3> &origin,
    const std::array<double, 3> &a,
    const std::array<double, 3> &b)
{
    return (a[0] - origin[0]) * (b[1] - origin[1]) -
           (a[1] - origin[1]) * (b[0] - origin[0]);
}

std::vector<std::array<double, 3>> convexHull2d(std::vector<std::array<double, 3>> points)
{
    std::sort(
        points.begin(),
        points.end(),
        [](const auto &a, const auto &b) {
            if (std::abs(a[0] - b[0]) > 1.0e-9)
            {
                return a[0] < b[0];
            }
            return a[1] < b[1];
        });
    points.erase(
        std::unique(
            points.begin(),
            points.end(),
            [](const auto &a, const auto &b) {
                return std::abs(a[0] - b[0]) < 1.0e-9 &&
                       std::abs(a[1] - b[1]) < 1.0e-9;
            }),
        points.end());
    if (points.size() <= 2)
    {
        return points;
    }

    std::vector<std::array<double, 3>> lower;
    for (const auto &point : points)
    {
        while (lower.size() >= 2 &&
               cross2d(lower[lower.size() - 2], lower.back(), point) <= 0.0)
        {
            lower.pop_back();
        }
        lower.push_back(point);
    }

    std::vector<std::array<double, 3>> upper;
    for (auto it = points.rbegin(); it != points.rend(); ++it)
    {
        while (upper.size() >= 2 &&
               cross2d(upper[upper.size() - 2], upper.back(), *it) <= 0.0)
        {
            upper.pop_back();
        }
        upper.push_back(*it);
    }

    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

void addSphere(
    mjvScene *scene,
    const std::array<double, 3> &pos,
    double radius,
    const float rgba[4])
{
    if (!scene || scene->ngeom >= scene->maxgeom)
    {
        return;
    }
    double size[3] = {radius, radius, radius};
    double mat[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    mjv_initGeom(
        &scene->geoms[scene->ngeom],
        mjGEOM_SPHERE,
        size,
        pos.data(),
        mat,
        rgba);
    ++scene->ngeom;
}

void addSegment(
    mjvScene *scene,
    const std::array<double, 3> &from,
    const std::array<double, 3> &to,
    double width,
    const float rgba[4])
{
    if (!scene || scene->ngeom >= scene->maxgeom)
    {
        return;
    }
    double size[3] = {width, 0.0, 0.0};
    double pos[3] = {0.0, 0.0, 0.0};
    double mat[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    mjvGeom *geom = &scene->geoms[scene->ngeom];
    mjv_initGeom(geom, mjGEOM_CAPSULE, size, pos, mat, rgba);
    mjv_connector(geom, mjGEOM_CAPSULE, width, from.data(), to.data());
    ++scene->ngeom;
}
} // namespace

void MujocoSimBridge::initializeComSupportVisualization()
{
    com_support_visualization_ready_ = false;
    com_pinocchio_data_.reset();
    com_pinocchio_joint_q_indices_.clear();
    support_foot_site_ids_.clear();

    if (!enable_com_support_visualization_)
    {
        return;
    }

    std::string urdf_path = trimCopy(com_support_pinocchio_urdf_path_);
    if (urdf_path.empty())
    {
        urdf_path = controller_runtime_.runtimeCfg().pinocchio_urdf_path;
    }
    if (urdf_path.empty())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "COM/support visualization requested but no Pinocchio URDF is configured.");
        return;
    }
    if (!std::filesystem::exists(urdf_path))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "COM/support visualization requested but Pinocchio URDF does not exist: %s",
            urdf_path.c_str());
        return;
    }

    try
    {
        pinocchio::urdf::buildModel(
            urdf_path,
            pinocchio::JointModelFreeFlyer(),
            com_pinocchio_model_);
        com_pinocchio_data_ = std::make_unique<pinocchio::Data>(com_pinocchio_model_);
        com_pinocchio_q_ = pinocchio::neutral(com_pinocchio_model_);

        for (const std::string &joint_name : joint_names_)
        {
            if (!com_pinocchio_model_.existJointName(joint_name))
            {
                throw std::runtime_error("Pinocchio model is missing joint '" + joint_name + "'");
            }
            const pinocchio::JointIndex joint_id = com_pinocchio_model_.getJointId(joint_name);
            const auto &joint_model = com_pinocchio_model_.joints[joint_id];
            if (joint_model.nq() != 1)
            {
                throw std::runtime_error("Pinocchio joint '" + joint_name + "' is not 1-DOF");
            }
            com_pinocchio_joint_q_indices_.push_back(joint_model.idx_q());
        }

        for (const std::string &site_name : support_foot_site_names_)
        {
            const int site_id = mj_name2id(model_, mjOBJ_SITE, site_name.c_str());
            if (site_id < 0)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Support foot site '%s' not found in MuJoCo model; ignoring it.",
                    site_name.c_str());
                continue;
            }
            support_foot_site_ids_.push_back(site_id);
        }
        if (support_foot_site_ids_.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "COM/support visualization requested but no support foot sites were resolved.");
        }

        com_support_visualization_ready_ = true;
        RCLCPP_INFO(
            this->get_logger(),
            "COM/support visualization enabled. urdf='%s', support_sites=%zu",
            urdf_path.c_str(),
            support_foot_site_ids_.size());
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Failed to initialize COM/support visualization: %s",
            e.what());
        com_pinocchio_data_.reset();
        com_pinocchio_joint_q_indices_.clear();
        support_foot_site_ids_.clear();
    }
}

MujocoSimBridge::ComSupportOverlay
MujocoSimBridge::computeComSupportOverlay(const mjData_ *data) const
{
    ComSupportOverlay overlay;
    if (!enable_com_support_visualization_ ||
        !com_support_visualization_ready_ ||
        !data ||
        !com_pinocchio_data_ ||
        com_pinocchio_joint_q_indices_.size() != joint_names_.size())
    {
        return overlay;
    }

    pinocchio::Model::ConfigVectorType q = com_pinocchio_q_;
    if (base_free_qpos_adr_ >= 0 && (base_free_qpos_adr_ + 6) < model_->nq)
    {
        q[0] = data->qpos[base_free_qpos_adr_ + 0];
        q[1] = data->qpos[base_free_qpos_adr_ + 1];
        q[2] = data->qpos[base_free_qpos_adr_ + 2];
        const auto quat_xyzw = normalizedXyzw(
            data->qpos[base_free_qpos_adr_ + 4],
            data->qpos[base_free_qpos_adr_ + 5],
            data->qpos[base_free_qpos_adr_ + 6],
            data->qpos[base_free_qpos_adr_ + 3]);
        q[3] = quat_xyzw[0];
        q[4] = quat_xyzw[1];
        q[5] = quat_xyzw[2];
        q[6] = quat_xyzw[3];
    }

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const int qpos_adr = (i < qpos_addrs_.size()) ? qpos_addrs_[i] : -1;
        const int pin_q_idx = com_pinocchio_joint_q_indices_[i];
        if (qpos_adr >= 0 && qpos_adr < model_->nq && pin_q_idx >= 0 && pin_q_idx < q.size())
        {
            q[pin_q_idx] = data->qpos[qpos_adr];
        }
    }

    try
    {
        pinocchio::Data pin_data(com_pinocchio_model_);
        const Eigen::Vector3d com = pinocchio::centerOfMass(
            com_pinocchio_model_,
            pin_data,
            q,
            false);
        overlay.com = {com.x(), com.y(), com.z()};
        overlay.valid = true;
    }
    catch (const std::exception &)
    {
        return ComSupportOverlay{};
    }

    double min_z = std::numeric_limits<double>::infinity();
    for (const int site_id : support_foot_site_ids_)
    {
        if (site_id >= 0 && site_id < model_->nsite)
        {
            min_z = std::min(min_z, data->site_xpos[3 * site_id + 2]);
        }
    }
    if (!std::isfinite(min_z))
    {
        return overlay;
    }

    std::vector<std::array<double, 3>> support_points;
    for (const int site_id : support_foot_site_ids_)
    {
        if (site_id < 0 || site_id >= model_->nsite)
        {
            continue;
        }
        const double *pos = &data->site_xpos[3 * site_id];
        if (pos[2] > min_z + support_contact_height_threshold_)
        {
            continue;
        }
        const double *mat = &data->site_xmat[9 * site_id];
        const std::array<double, 3> x_axis{mat[0], mat[3], mat[6]};
        const std::array<double, 3> y_axis{mat[1], mat[4], mat[7]};
        for (const double sx : {-1.0, 1.0})
        {
            for (const double sy : {-1.0, 1.0})
            {
                support_points.push_back({
                    pos[0] + sx * support_foot_half_length_ * x_axis[0] + sy * support_foot_half_width_ * y_axis[0],
                    pos[1] + sx * support_foot_half_length_ * x_axis[1] + sy * support_foot_half_width_ * y_axis[1],
                    0.018});
            }
        }
    }
    overlay.support_polygon = convexHull2d(std::move(support_points));
    for (auto &point : overlay.support_polygon)
    {
        point[2] = 0.018;
    }
    return overlay;
}

void MujocoSimBridge::appendComSupportOverlay(
    mjvScene *scene,
    const ComSupportOverlay &overlay) const
{
    if (!scene || !overlay.valid)
    {
        return;
    }

    constexpr float kComRgba[4] = {1.0f, 0.18f, 0.05f, 1.0f};
    constexpr float kProjectionRgba[4] = {0.05f, 0.45f, 1.0f, 1.0f};
    constexpr float kSupportRgba[4] = {0.05f, 0.85f, 0.30f, 0.95f};

    std::array<double, 3> projection{overlay.com[0], overlay.com[1], 0.02};
    addSphere(scene, overlay.com, com_marker_radius_, kComRgba);
    addSphere(scene, projection, com_projection_marker_radius_, kProjectionRgba);
    addSegment(scene, overlay.com, projection, 0.006, kComRgba);

    if (overlay.support_polygon.size() >= 2)
    {
        for (size_t i = 0; i < overlay.support_polygon.size(); ++i)
        {
            const auto &from = overlay.support_polygon[i];
            const auto &to = overlay.support_polygon[(i + 1) % overlay.support_polygon.size()];
            addSegment(scene, from, to, 0.008, kSupportRgba);
            addSphere(scene, from, 0.018, kSupportRgba);
        }
    }
}

} // namespace mujoco_sim2sim
