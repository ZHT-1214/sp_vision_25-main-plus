#include "transform_tools/transform_tools.h"
#include <opencv2/core/eigen.hpp>

namespace transform_tools
{

Eigen::Matrix3d SingleEulerAngle::to_rotation_mat() const
{
    switch (m_rotation_axis)
    {
    case RotationAxis::X:
        return Eigen::AngleAxisd(m_angle, Eigen::Vector3d::UnitX()).matrix();
    case RotationAxis::Y:
        return Eigen::AngleAxisd(m_angle, Eigen::Vector3d::UnitY()).matrix();
    case RotationAxis::Z:
        return Eigen::AngleAxisd(m_angle, Eigen::Vector3d::UnitZ()).matrix();
    }

    return Eigen::Matrix3d::Identity();
}

Yaw::Yaw(const Angle &angle, RotationAxis axis)
    : SingleEulerAngle(angle, axis)
{
}

Roll::Roll(const Angle &angle, RotationAxis axis)
    : SingleEulerAngle(angle, axis)
{
}

Pitch::Pitch(const Angle &angle, RotationAxis axis)
    : SingleEulerAngle(angle, axis)
{
}

TF::TF()
{
}

TF::TF(const Sophus::SE3d &transfer)
{
    m_transfer = transfer;
}

const Sophus::SE3d &TF::get_transfer() const
{
    return m_transfer;
}

void TF::set_offset(const Eigen::Vector3d &offset)
{
    m_transfer.translation() = offset;
}

Eigen::Vector3d TF::get_offset() const
{
    return m_transfer.translation();
}

cv::Vec3d TF::get_cv_offset() const
{
    Eigen::Vector3d offset = get_offset();
    cv::Vec3d result;
    cv::eigen2cv(offset, result);
    return result;
}

Eigen::Matrix3d TF::get_rotation() const
{
    return m_transfer.rotationMatrix();
}

cv::Vec3d TF::get_cv_rotation() const
{
    Eigen::Matrix3d rotation = get_rotation();
    cv::Mat cv_rotation_mat;
    cv::eigen2cv(rotation, cv_rotation_mat);
    cv::Vec3d result;
    cv::Rodrigues(cv_rotation_mat, result);
    return result;
}

void TF::set_rotation(const Eigen::Matrix3d &rotation_mat)
{
    m_transfer.setRotationMatrix(rotation_mat);
}

TF TF::inversed() const
{
    return TF(m_transfer.inverse());
}

void TF::inverse()
{
    m_transfer = m_transfer.inverse();
}

Eigen::Vector3d TF::operator*(const Eigen::Vector3d &point) const
{
    return m_transfer * point;
}

TF TF::operator*(const TF &other) const
{
    return TF(m_transfer * other.m_transfer);
}

TF &TF::operator*=(const TF &other)
{
    m_transfer *= other.m_transfer;
    return *this;
}

Eigen::Vector3d TF::transform(const Eigen::Vector3d &point) const
{
    return m_transfer * point;
}

Yaw TF::calculate_yaw() const
{
    Eigen::Matrix3d R = get_rotation();
    return Yaw(Angle::from_rad(std::atan2(R(0, 2), R(2, 2))));
}

Pitch TF::calculate_pitch() const
{
    Eigen::Matrix3d R = get_rotation();
    return Pitch(Angle::from_rad(std::asin(-R(1, 2))));
}

Roll TF::calculate_roll() const
{
    Eigen::Matrix3d R = get_rotation();
    return Roll(Angle::from_rad(std::atan2(R(1, 0), R(1, 1))));
}

TFNode::TFNode(TF parent_T_this, FrameID frame_id, TFNode *parent_node)
    : parent_T_this(parent_T_this), frame_id(frame_id), parent_node(parent_node)
{
}

TFNode::~TFNode()
{
    for (const auto *p_child : children)
        delete p_child;
}

TFTree::TFTree(FrameID frame_id)
    : m_base(TF(), frame_id, nullptr)
{
    m_id_map[frame_id] = &m_base;
}

TFTree::TFTree(const TFTree &tree)
    : TFTree(tree.m_base.frame_id)
{
    for (const auto child : tree.m_base.children)
        add_subtree(*child);
}

TFTree::~TFTree()
{
}

TFTree &TFTree::operator=(const TFTree &other)
{
    // 完全清除本树的数据
    for (const auto &child : m_base.children)
        delete child;
    m_base.children.clear();
    m_id_map.clear();

    // 本树初始化
    m_base.parent_T_this = TF();
    m_base.parent_node = nullptr;
    m_base.frame_id = other.m_base.frame_id;
    m_id_map[m_base.frame_id] = &m_base;

    // 构建子树
    for (const auto child : other.m_base.children)
        add_subtree(*child);

    return *this;
}

TFNode *TFTree::find(FrameID frame_id) const
{
    auto iter = m_id_map.find(frame_id);
    if (iter == m_id_map.end())
        return nullptr;
    else
        return iter->second;
}

void TFTree::add_TF(const TF &parent_T_this, FrameID parent_id, FrameID frame_id)
{
    if (find(frame_id))
        throw std::runtime_error("The TF with same name has already existed!");

    TFNode *parent_node = find(parent_id);
    if (!parent_node)
        throw std::runtime_error("Error parent frame ID!");

    TFNode *node = new TFNode(parent_T_this, frame_id, parent_node);
    parent_node->children.push_back(node);
    m_id_map[frame_id] = node;
}

void TFTree::remove(FrameID frame_id)
{
    TFNode *node = find(frame_id);
    if (!node)
        throw std::runtime_error("Error frame ID!");
    if (node->frame_id == m_base.frame_id)
        throw std::runtime_error("Failed to remove the base TF!");

    remove_from_map(frame_id);

    TFNode *parent = node->parent_node;
    auto iter = std::find(parent->children.begin(), parent->children.end(), node);
    parent->children.erase(iter);

    delete node;
}

TF TFTree::calculate_absolute_TF(FrameID frame_id) const
{
    TFNode *node = find(frame_id);
    if (!node)
    {
        std::cerr << "[TFTree] calculate_absolute_TF: requested frame id=" << frame_id << " not found. Available frames:";
        for (const auto &p : m_id_map)
            std::cerr << ' ' << p.first;
        std::cerr << std::endl;
        throw std::runtime_error("Error frame ID!");
    }

    return calculate_absolute_TF_impl(node, TF());
}

TF TFTree::calculate_relative_TF(FrameID start_frame_id, FrameID target_frame_id) const
{
    TF base_T_start = calculate_absolute_TF(start_frame_id);
    TF base_T_target = calculate_absolute_TF(target_frame_id);
    return base_T_start.inversed() * base_T_target;
}

TF &TFTree::operator[](FrameID frame_id)
{
    return get(frame_id);
}

TF &TFTree::get(FrameID frame_id)
{
    TFNode *node = find(frame_id);
    if (!node)
    {
        std::cerr << "[TFTree] get: requested frame id=" << frame_id << " not found. Available frames:";
        for (const auto &p : m_id_map)
            std::cerr << ' ' << p.first;
        std::cerr << std::endl;
        throw std::runtime_error("Error frame ID!");
    }
    if (node->frame_id == m_base.frame_id)
        throw std::runtime_error("Failed to operate the base frame");

    return node->parent_T_this;
}

const TF &TFTree::operator[](FrameID frame_id) const
{
    return get(frame_id);
}

const TF &TFTree::get(FrameID frame_id) const
{
    TFNode *node = find(frame_id);
    if (!node)
    {
        std::cerr << "[TFTree] get(const): requested frame id=" << frame_id << " not found. Available frames:";
        for (const auto &p : m_id_map)
            std::cerr << ' ' << p.first;
        std::cerr << std::endl;
        throw std::runtime_error("Error frame ID!");
    }
    if (node->frame_id == m_base.frame_id)
        throw std::runtime_error("Failed to operate the base frame");

    return node->parent_T_this;
}

void TFTree::remove_from_map(FrameID frame_id)
{
    TFNode *node = find(frame_id);

    m_id_map.erase(frame_id);
    for (const auto child : node->children)
        remove_from_map(child->frame_id);
}

TF TFTree::calculate_absolute_TF_impl(TFNode *node, const TF &tf_to_node_child) const
{
    if (node == &m_base)
        return m_base.parent_T_this * tf_to_node_child;
    else
        return calculate_absolute_TF_impl(node->parent_node, node->parent_T_this * tf_to_node_child);
}

void TFTree::add_subtree(const TFNode &subtree_root)
{
    // 利用 subtree_root 中的坐标系 ID 信息（包括出发坐标系和目标坐标系）在本 TFTree 中构建子树
    add_TF(subtree_root.parent_T_this, subtree_root.parent_node->frame_id, subtree_root.frame_id);

    for (const auto child : subtree_root.children)
        add_subtree(*child);
}

Eigen::Vector3d cv2eigen(const cv::Point3d &p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}
Eigen::Vector3d cv2eigen(const cv::Vec3d &v)
{
    return Eigen::Vector3d(v[0], v[1], v[2]);
}
Eigen::Vector2d cv2eigen(const cv::Point2d &p)
{
    return Eigen::Vector2d(p.x, p.y);
}
Eigen::Vector2d cv2eigen(const cv::Vec2d &v)
{
    return Eigen::Vector2d(v[0], v[1]);
}
Eigen::Matrix3d cv2eigen(const cv::Mat &m)
{
    Eigen::Matrix3d result;
    cv::cv2eigen(m, result);
    return result;
}
Eigen::Matrix3d cv2eigen(const cv::Matx33d &m)
{
    Eigen::Matrix3d result;
    cv::cv2eigen(m, result);
    return result;
}
cv::Vec3d eigen2cv(const Eigen::Vector3d &v)
{
    return cv::Vec3d(v.x(), v.y(), v.z());
}
cv::Vec2d eigen2cv(const Eigen::Vector2d &v)
{
    return cv::Vec2d(v.x(), v.y());
}
cv::Mat eigen2cv(const Eigen::Matrix3d &m)
{
    cv::Mat result;
    cv::eigen2cv(m, result);
    return result;
}

} // transform_tools