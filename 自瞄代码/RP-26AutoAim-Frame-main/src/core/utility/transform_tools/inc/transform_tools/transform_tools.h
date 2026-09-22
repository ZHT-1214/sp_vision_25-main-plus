#pragma once
#include <sophus/se3.hpp>
#include <numbers>
#include <cmath>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/core/matx.hpp>
#include "frame_id.h"

namespace transform_tools
{

/// @brief 在自瞄代码中，有着三套旋转度量方式，分别是角度制、弧度制以及电控角度制（0-8192），角度制用以调整算法参数，
/// 弧度制用以进行三角函数运算，电控角度制用于与电控通信。故为了使得角度转换规范化，保证转换正确性，设计了 Angle 类。
/// Angle 类目标是，将三种旋转度量方式视为“同一种东西”，提供一套统一的运算接口，方便进行角度的操作。
/// @note 该类支持编译期计算    
class Angle
{
public:
    /// @brief 将一个在 R 上取值的弧度量归一化到 [-pi, pi) 上
    /// @param angle 需要归一化的弧度量
    /// @return 归一化结果
    constexpr static double limit_rad(double angle)
    {
        double n = std::floor((angle + pi) / pi_2);
        return angle - n * pi_2;
    }

    /// @brief 从一个弧度量构造 Angle 对象
    /// @param angle 构造所需的弧度量
    /// @return 所构造的 Angle 对象
    constexpr static Angle from_rad(double angle)
    { return Angle(limit_rad(angle)); }
    
    /// @brief 从一个角度量构造 Angle 对象
    /// @param angle 构造所需的角度量
    /// @return 所构造的 Angle 对象
    constexpr static Angle from_degree(double angle)
    {
        angle = angle * std::numbers::pi_v<double> / 180.0;
        return Angle(limit_rad(angle));
    }
    /// @brief 从一个电控角度量构造 Angle 对象
    /// @param angle 构造所需的电控角度量
    /// @return 所构造的 Angle 对象
    constexpr static Angle from_ECS_degree(double angle)
    {
        angle = (angle - 4096.0) * 2.0 * std::numbers::pi_v<double> / 8192.0;
        return Angle(limit_rad(angle));
    }

    /// @brief 将 Angle 转换为弧度量
    constexpr double to_rad() const
    { return m_angle; }
    /// @brief 将 Angle 转换为角度量
    constexpr double to_degree() const
    { return m_angle * 180.0 / std::numbers::pi_v<double>; }
    /// @brief 将 Angle 转换为电控角度量
    constexpr double to_ECS() const
    { return m_angle * 8192.0 / (2.0 * std::numbers::pi_v<double>) + 4096.0; }
    
    /// @brief 将一个角的终边关于原点对称翻转
    constexpr Angle operator-() const
    { return Angle(-m_angle); }

    constexpr Angle operator-(const Angle &other) const
    { return Angle(limit_rad(m_angle - other.m_angle)); }
    constexpr Angle& operator-=(const Angle &other)
    {
        m_angle = limit_rad(m_angle - other.m_angle);
        return *this;
    }
    constexpr Angle operator+(const Angle &other) const
    { return Angle(limit_rad(m_angle + other.m_angle)); }
    constexpr Angle& operator+=(const Angle &other)
    {
        m_angle = limit_rad(m_angle + other.m_angle);
        return *this;
    }

    constexpr Angle operator*(double val) const
    { return Angle(limit_rad(m_angle * val)); }
    constexpr friend Angle operator*(double val, const Angle &angle)
    { return angle * val; }
    constexpr Angle& operator*=(double val)
    {
        m_angle = limit_rad(m_angle * val);
        return *this;
    }

protected:
    constexpr Angle(double angle)
        : m_angle(angle)
    { }
    double m_angle;
    constexpr static double pi = std::numbers::pi_v<double>;
    constexpr static double pi_2 = 2.0 * std::numbers::pi_v<double>;
};

enum class RotationAxis
{
    X,
    Y,
    Z
};

/// @brief 该类是一个抽象类，不可实例化，代表三种绕轴旋转（Yaw轴、Pitch轴、Roll轴）中的任意一种
class SingleEulerAngle : public Angle
{
public:
    /// @brief 依据旋转量大小进行构造
    /// @param angle 构造所需的旋转量
    inline SingleEulerAngle(const Angle &angle, RotationAxis axis)
        : Angle(angle), m_rotation_axis(axis) { }
    /// @brief 计算绕轴旋转的旋转矩阵
    /// @return 绕轴旋转的旋转矩阵
    virtual Eigen::Matrix3d to_rotation_mat() const;

protected:
    RotationAxis m_rotation_axis;
};

template <typename T>
concept SingleEulerAngleType = std::is_base_of_v<SingleEulerAngle, std::decay_t<T>>;

/// @brief 该类代表一个绕 Yaw 轴的旋转
class Yaw : public SingleEulerAngle
{
public:
    /// @brief 依据旋转量大小进行构造
    /// @param angle 构造所需的旋转量
    Yaw(const Angle &angle, RotationAxis axis = RotationAxis::Y);
};

/// @brief 该类代表一个绕 Roll 轴的旋转
class Roll : public SingleEulerAngle
{
public:
    /// @brief 依据旋转量大小进行构造
    /// @param angle 构造所需的旋转量
    Roll(const Angle &angle, RotationAxis axis = RotationAxis::Z);
};

/// @brief 该类代表一个绕 Pitch 轴的旋转
class Pitch : public SingleEulerAngle
{
public:
    /// @brief 依据旋转量大小进行构造
    /// @param angle 构造所需的旋转量
    Pitch(const Angle &angle, RotationAxis axis = RotationAxis::X);
};

class TF
{
public:
    TF();

    template<SingleEulerAngleType... T>
    TF(
        const Eigen::Vector3d &offset = Eigen::Vector3d::Zero(),
        T&&... euler_angles
    )
    { setTF(offset, euler_angles...); }

    TF(const Sophus::SE3d& transfer);

    template<SingleEulerAngleType... T>
    void setTF(
        const Eigen::Vector3d &offset,
        T&&... euler_angles
    )
    {
        set_offset(offset);
        if constexpr (sizeof...(T) != 0)
            set_rotation(euler_angles...);
    }
    const Sophus::SE3d& get_transfer() const;
    
    void set_offset(const Eigen::Vector3d &offset);
    Eigen::Vector3d get_offset() const;
    cv::Vec3d get_cv_offset() const;

    template<SingleEulerAngleType... T>
    void set_rotation(T&&... euler_angles)
    {
        Eigen::Matrix3d R = (euler_angles.to_rotation_mat() * ...);
        set_rotation(R);
    }
    void set_rotation(const Eigen::Matrix3d &rotation_mat);
    Eigen::Matrix3d get_rotation() const;
    cv::Vec3d get_cv_rotation() const;

    TF inversed() const;
    void inverse();

    Eigen::Vector3d operator*(const Eigen::Vector3d &point) const;
    TF operator*(const TF &other) const;
    TF& operator*=(const TF &other);

    Eigen::Vector3d transform(const Eigen::Vector3d &point) const;

    Yaw calculate_yaw() const;
    Pitch calculate_pitch() const;
    Roll calculate_roll() const;

private:
    Sophus::SE3d m_transfer;
};

struct TFNode
{
    TFNode(TF parent_T_this, FrameID frame_id, TFNode* parent_node);
    ~TFNode();
    TF parent_T_this;
    FrameID frame_id;
    TFNode* parent_node;
    std::vector<TFNode*> children;
};

class TFTree
{
public:
    TFTree(FrameID frame_id = car_frame);
    TFTree(const TFTree& tree);
    TFTree(TFTree &&) = delete;
    ~TFTree();

    TFTree& operator= (const TFTree &other);

    TFNode* find(FrameID frame_id) const;
    void add_TF(const TF &parent_T_this, FrameID parent_id, FrameID frame_id);
    void remove(FrameID frame_id);

    TF calculate_absolute_TF(FrameID frame_id) const;
    TF calculate_relative_TF(FrameID start_frame_id, FrameID target_frame_id) const;

    TF& operator[](FrameID frame_id);

    TF& get(FrameID frame_id);

    const TF& operator[](FrameID frame_id) const;
    const TF& get(FrameID frame_id) const;

private:
    void remove_from_map(FrameID frame_id);
    TF calculate_absolute_TF_impl(TFNode* node, const TF &accumulated_tf) const;
    void add_subtree(const TFNode &subtree_root);

private:
    TFNode m_base;
    std::map<FrameID, TFNode*> m_id_map;
};

Eigen::Vector3d cv2eigen(const cv::Point3d&);
Eigen::Vector3d cv2eigen(const cv::Vec3d&);
Eigen::Vector2d cv2eigen(const cv::Point2d&);
Eigen::Vector2d cv2eigen(const cv::Vec2d&);
Eigen::Matrix3d cv2eigen(const cv::Mat&);
Eigen::Matrix3d cv2eigen(const cv::Matx33d&);
cv::Vec3d eigen2cv(const Eigen::Vector3d&);
cv::Vec2d eigen2cv(const Eigen::Vector2d&);
cv::Mat eigen2cv(const Eigen::Matrix3d&);

} // namespace transform_tools

namespace transform_tools::literals
{

/// 以下是字面量函数，能方便地构造一个 Angle 对象，如 90_deg 代表一个通过 90° 构造的 Angle 对象

inline constexpr Angle operator "" _rad(long double angle)
{
    return Angle::from_rad(static_cast<double>(angle));
}

inline constexpr Angle operator "" _deg(long double angle)
{
    return Angle::from_degree(static_cast<double>(angle));
}

inline constexpr Angle operator "" _rad(unsigned long long angle)
{
    return Angle::from_rad(static_cast<double>(angle));
}

inline constexpr Angle operator "" _deg(unsigned long long angle)
{
    return Angle::from_degree(static_cast<double>(angle));
}

} // namespace transform_tools::literals