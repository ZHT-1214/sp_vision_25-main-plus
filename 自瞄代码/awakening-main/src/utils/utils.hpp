#pragma once
#include "angles.h"
#include "utils/common/type_common.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <ceres/jet.h>
#include <cmath>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <pwd.h>
#include <regex>
#include <utility>
#include <vector>
namespace awakening::utils {
template<class T>
inline Eigen::Matrix<T, 3, 3> so3_hat(const Eigen::Matrix<T, 3, 1>& w) {
    Eigen::Matrix<T, 3, 3> W;
    W << T(0), -w.z(), w.y(), w.z(), T(0), -w.x(), -w.y(), w.x(), T(0);
    return W;
}
template<class T>
inline Eigen::Matrix<T, 3, 3> so3_exp(const Eigen::Matrix<T, 3, 1>& phi) {
    const T theta2 = phi.squaredNorm();
    const T theta = ceres::sqrt(theta2);

    const auto W = so3_hat(phi);
    const auto W2 = W * W;

    Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity();

    T A;
    T B;
    if (theta2 < T(1e-12)) {
        const T theta4 = theta2 * theta2;
        A = T(1) - theta2 / T(6) + theta4 / T(120);
        B = T(0.5) - theta2 / T(24) + theta4 / T(720);
    } else {
        A = ceres::sin(theta) / theta;
        B = (T(1) - ceres::cos(theta)) / theta2;
    }

    R += A * W + B * W2;

    return R;
}

template<class T>
inline Eigen::Transform<T, 3, Eigen::Isometry>
se3_exp(const Eigen::Matrix<T, 3, 1>& rho, const Eigen::Matrix<T, 3, 1>& phi) {
    const T theta2 = phi.squaredNorm();
    const T theta = ceres::sqrt(theta2);

    const auto W = so3_hat(phi);
    const auto W2 = W * W;

    T B;
    T C;
    if (theta2 < T(1e-12)) {
        const T theta4 = theta2 * theta2;
        B = T(0.5) - theta2 / T(24) + theta4 / T(720);
        C = T(1.0 / 6.0) - theta2 / T(120) + theta4 / T(5040);
    } else {
        B = (T(1) - ceres::cos(theta)) / theta2;
        C = (theta - ceres::sin(theta)) / (theta2 * theta);
    }

    const Eigen::Matrix<T, 3, 3> V = Eigen::Matrix<T, 3, 3>::Identity() + B * W + C * W2;

    Eigen::Transform<T, 3, Eigen::Isometry> T_se3 =
        Eigen::Transform<T, 3, Eigen::Isometry>::Identity();
    T_se3.linear() = so3_exp(phi);
    T_se3.translation() = V * rho;
    return T_se3;
}
template<class T>
inline Eigen::Matrix<T, 3, 1> so3_log(const Eigen::Matrix<T, 3, 3>& R) {
    const T cos_theta = (R.trace() - T(1.0)) * T(0.5);
    Eigen::Matrix<T, 3, 1> w;
    w << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
    if (cos_theta > T(1.0 - 1e-12)) {
        return T(0.5) * w;
    }
    const T theta = ceres::acos(cos_theta);
    const T scale = theta / (T(2.0) * ceres::sin(theta));
    return scale * w;
}
template<class T>
inline void se3_log(
    const Eigen::Transform<T, 3, Eigen::Isometry>& T_se3,
    Eigen::Matrix<T, 3, 1>& rho,
    Eigen::Matrix<T, 3, 1>& phi
) {
    phi = so3_log(T_se3.linear().eval());

    const T theta2 = phi.squaredNorm();
    const T theta = ceres::sqrt(theta2);
    const auto W = so3_hat(phi);
    const auto W2 = W * W;

    T D;
    if (theta2 < T(1e-12)) {
        const T theta4 = theta2 * theta2;
        D = T(1.0 / 12.0) + theta2 / T(720) + theta4 / T(30240);
    } else {
        D = T(1) / theta2 - (T(1) + ceres::cos(theta)) / (T(2) * theta * ceres::sin(theta));
    }

    const Eigen::Matrix<T, 3, 3> V_inv = Eigen::Matrix<T, 3, 3>::Identity() - T(0.5) * W + D * W2;
    rho = V_inv * T_se3.translation();
}
enum class RPYOrder { XYZ, ZYX };
template<class T>
inline Eigen::Quaternion<T> rpy2quat(const Eigen::Vector3<T>& rpy, RPYOrder order = RPYOrder::ZYX) {
    Eigen::AngleAxis<T> roll(rpy.x(), Eigen::Vector3<T>::UnitX());
    Eigen::AngleAxis<T> pitch(rpy.y(), Eigen::Vector3<T>::UnitY());
    Eigen::AngleAxis<T> yaw(rpy.z(), Eigen::Vector3<T>::UnitZ());
    Eigen::Quaternion<T> q;
    switch (order) {
        case RPYOrder::ZYX:
            q = { yaw * pitch * roll };
            break;
        case RPYOrder::XYZ:
            q = { roll * pitch * yaw };
            break;
    }
    q.normalize();
    return q;
}

template<class T>
inline Eigen::Matrix3<T> rpy2matrix(const Eigen::Vector3<T>& rpy, RPYOrder order = RPYOrder::ZYX) {
    return rpy2quat(rpy, order).toRotationMatrix();
}

template<class T>
inline Eigen::Vector3<T> matrix2rpy(const Eigen::Matrix3<T>& R, RPYOrder order = RPYOrder::ZYX) {
    switch (order) {
        case RPYOrder::ZYX: {
            const T roll = ceres::atan2(R(2, 1), R(2, 2));
            const T pitch = ceres::atan2(-R(2, 0), ceres::hypot(R(2, 1), R(2, 2)));
            const T yaw = ceres::atan2(R(1, 0), R(0, 0));
            return { roll, pitch, yaw };
        }

        case RPYOrder::XYZ: {
            const T pitch = ceres::asin(-R(0, 2));
            const T roll = ceres::atan2(R(1, 2), R(2, 2));
            const T yaw = ceres::atan2(R(0, 1), R(0, 0));
            return { roll, pitch, yaw };
        }
    }

    return { T(0), T(0), T(0) };
}

template<class T>
inline Eigen::Vector3<T> quat2rpy(const Eigen::Quaternion<T>& q, RPYOrder order = RPYOrder::ZYX) {
    return matrix2rpy(q.normalized().toRotationMatrix(), order);
}

inline std::string expand_env(const std::string& s) {
    std::regex env_re(R"(\$\{([^}]+)\})");
    std::smatch match;
    std::string result = s;
    while (std::regex_search(result, match, env_re)) {
        const char* env = std::getenv(match[1].str().c_str());
        std::string val = env ? env : "";
        result.replace(match.position(0), match.length(0), val);
    }
    return result;
}
template<typename Func>
void dt_once(Func&& func, std::chrono::duration<double> dt) noexcept {
    static auto last_call = std::chrono::steady_clock::now();

    auto now = std::chrono::steady_clock::now();
    if (now - last_call >= dt) {
        last_call = now;
        func();
    }
}
template<typename T>
concept Point2DLike = requires(T p) {
    {
        p.x
        } -> std::convertible_to<float>;
    {
        p.y
        } -> std::convertible_to<float>;
    T { 0.f, 0.f };
};
template<Point2DLike T>
[[nodiscard]] inline T transform_point2D(const Eigen::Matrix3f& H, const T& p) noexcept {
    const Eigen::Vector3f hp { p.x, p.y, 1.f };
    const Eigen::Vector3f tp = H * hp;
    return { tp.x(), tp.y() };
}
inline cv::Rect2f transform_rect(const Eigen::Matrix3f& H, const cv::Rect2f& rect) {
    cv::Point2f p1(rect.x, rect.y);
    cv::Point2f p2(rect.x + rect.width, rect.y);
    cv::Point2f p3(rect.x, rect.y + rect.height);
    cv::Point2f p4(rect.x + rect.width, rect.y + rect.height);

    auto tp1 = utils::transform_point2D(H, p1);
    auto tp2 = utils::transform_point2D(H, p2);
    auto tp3 = utils::transform_point2D(H, p3);
    auto tp4 = utils::transform_point2D(H, p4);

    float min_x = std::min({ tp1.x, tp2.x, tp3.x, tp4.x });
    float min_y = std::min({ tp1.y, tp2.y, tp3.y, tp4.y });
    float max_x = std::max({ tp1.x, tp2.x, tp3.x, tp4.x });
    float max_y = std::max({ tp1.y, tp2.y, tp3.y, tp4.y });

    return cv::Rect2f(min_x, min_y, max_x - min_x, max_y - min_y);
}
inline cv::Mat letterbox(
    const cv::Mat& img,
    Eigen::Matrix3f& transform_matrix,
    const int new_shape_w,
    const int new_shape_h
) noexcept {
    const int img_h = img.rows;
    const int img_w = img.cols;

    const float scale = std::min((float)new_shape_h / img_h, (float)new_shape_w / img_w);
    const int resize_h = int(img_h * scale + 0.5f);
    const int resize_w = int(img_w * scale + 0.5f);

    const int pad_h = new_shape_h - resize_h;
    const int pad_w = new_shape_w - resize_w;
    const int top = pad_h / 2;
    const int left = pad_w / 2;

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(resize_w, resize_h), 0, 0, cv::INTER_LINEAR);

    cv::Mat out;
    if (pad_h == 0 && pad_w == 0) {
        out = resized;
    } else {
        cv::copyMakeBorder(
            resized,
            out,
            top,
            pad_h - top,
            left,
            pad_w - left,
            cv::BORDER_CONSTANT,
            cv::Scalar(114, 114, 114)
        );
    }

    const float inv_scale = 1.0f / scale;

    transform_matrix << inv_scale, 0, -left * inv_scale, 0, inv_scale, -top * inv_scale, 0, 0, 1;

    return out;
}
inline std::string to_upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
    return s;
}
template<std::size_t N1, std::size_t N2>
consteval auto concat(const char (&a)[N1], const char (&b)[N2]) {
    std::array<char, N1 + N2 - 1> result {}; // -1 是因为两个字面量都有 '\0'
    for (std::size_t i = 0; i < N1 - 1; ++i)
        result[i] = a[i];
    for (std::size_t i = 0; i < N2; ++i)
        result[i + N1 - 1] = b[i]; // 包含 '\0'
    return result;
}
template<typename T>
inline T from_vector(const std::vector<uint8_t>& data) {
    T packet {};
    std::memcpy(&packet, data.data(), sizeof(T));
    return packet;
}

template<typename T>
inline std::vector<uint8_t> to_vector(const T& data) {
    std::vector<uint8_t> packet(sizeof(T));
    std::memcpy(packet.data(), &data, sizeof(T));
    return packet;
}
inline std::pair<cv::Mat, cv::Mat> eigen_iso3_to_rvec_tvec(const ISO3& iso3) {
    cv::Mat rvec, R_cv;
    Mat3 R = iso3.linear();
    cv::eigen2cv(R, R_cv);
    cv::Rodrigues(R_cv, rvec);
    auto t = iso3.translation();
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << t.x(), t.y(), t.z());
    return { std::move(rvec), std::move(tvec) };
}
inline ISO3 rvec_tvec_to_eigen_iso3(const cv::Mat& rvec, const cv::Mat& tvec) {
    ISO3 iso3;
    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    Mat3 R_eigen;
    cv::cv2eigen(R_cv, R_eigen);
    iso3.linear() = R_eigen;
    Vec3 t_eigen;
    cv::cv2eigen(tvec, t_eigen);
    iso3.translation() = t_eigen;
    return iso3;
}
inline std::vector<cv::Point2f> reprojection(
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const std::vector<cv::Point3f>& object_points,
    const ISO3& pose_in_camera_cv
) noexcept {
    auto [rvec, tvec] = eigen_iso3_to_rvec_tvec(pose_in_camera_cv);
    std::vector<cv::Point2f> pts_2d;
    pts_2d.reserve(object_points.size());
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, pts_2d);
    return pts_2d;
}
template<typename ImgPoints>
inline ImgPoints undistort_points(
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const ImgPoints& img_points
) {
    ImgPoints norm_pts;
    cv::undistortPoints(img_points, norm_pts, camera_matrix, dist_coeffs);
    return norm_pts;
}
template<typename ImgPoint>
inline ImgPoint undistort_point(
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const ImgPoint& img_point
) {
    std::vector<ImgPoint> norm_pts;
    cv::undistortPoints(std::vector<ImgPoint> { img_point }, norm_pts, camera_matrix, dist_coeffs);
    return norm_pts.front();
}
template<typename T>
inline void project_points_jets(
    const std::vector<cv::Point3f>& obj_pts,
    const Eigen::Transform<T, 3, Eigen::Isometry>& pose_cam,
    const cv::Mat& K,
    const cv::Mat& dist_coeffs,
    std::vector<Eigen::Matrix<T, 2, 1>>& img_pts_jet
) {
    if (obj_pts.empty())
        return;
    if (K.empty() || K.rows != 3 || K.cols != 3)
        throw std::runtime_error("Invalid K");
    if (dist_coeffs.empty())
        throw std::runtime_error("Invalid dist_coeffs");

    const Eigen::Matrix<T, 3, 3>& R = pose_cam.linear();
    const Eigen::Matrix<T, 3, 1>& t = pose_cam.translation();

    const T fx = T(K.at<double>(0, 0));
    const T fy = T(K.at<double>(1, 1));
    const T cx = T(K.at<double>(0, 2));
    const T cy = T(K.at<double>(1, 2));

    auto get_dist = [&](int i) -> double {
        return (dist_coeffs.rows == 1) ? dist_coeffs.at<double>(0, i)
                                       : dist_coeffs.at<double>(i, 0);
    };

    const int n_dist = dist_coeffs.rows * dist_coeffs.cols;
    const T k1 = n_dist > 0 ? T(get_dist(0)) : T(0);
    const T k2 = n_dist > 1 ? T(get_dist(1)) : T(0);
    const T p1 = n_dist > 2 ? T(get_dist(2)) : T(0);
    const T p2 = n_dist > 3 ? T(get_dist(3)) : T(0);
    const T k3 = n_dist > 4 ? T(get_dist(4)) : T(0);

    img_pts_jet.clear();
    img_pts_jet.reserve(obj_pts.size());

    for (const auto& pt3: obj_pts) {
        Eigen::Matrix<T, 3, 1> Pw(T(pt3.x), T(pt3.y), T(pt3.z));
        Eigen::Matrix<T, 3, 1> Pc = R * Pw + t;
        T Xc = Pc(0), Yc = Pc(1), Zc = Pc(2);
        T xp = Xc / Zc;
        T yp = Yc / Zc;

        T r2 = xp * xp + yp * yp;
        T r4 = r2 * r2;
        T r6 = r4 * r2;

        T radial = T(1) + k1 * r2 + k2 * r4 + k3 * r6;
        T xd = xp * radial + T(2) * p1 * xp * yp + p2 * (r2 + T(2) * xp * xp);
        T yd = yp * radial + p1 * (r2 + T(2) * yp * yp) + T(2) * p2 * xp * yp;

        T u = fx * xd + cx;
        T v = fy * yd + cy;

        img_pts_jet.emplace_back(u, v);
    }
}
template<typename T>
inline void project_points_jets_normalized(
    const std::vector<cv::Point3f>& obj_pts,
    const Eigen::Transform<T, 3, Eigen::Isometry>& pose_cam,
    const cv::Mat& dist_coeffs,
    std::vector<Eigen::Matrix<T, 2, 1>>& norm_pts
) {
    if (obj_pts.empty())
        return;

    if (dist_coeffs.empty())
        throw std::runtime_error("Invalid dist_coeffs");

    const Eigen::Matrix<T, 3, 3>& R = pose_cam.linear();
    const Eigen::Matrix<T, 3, 1>& t = pose_cam.translation();

    auto get_dist = [&](int i) -> double {
        return (dist_coeffs.rows == 1) ? dist_coeffs.at<double>(0, i)
                                       : dist_coeffs.at<double>(i, 0);
    };

    const int n_dist = dist_coeffs.rows * dist_coeffs.cols;

    const T k1 = n_dist > 0 ? T(get_dist(0)) : T(0);
    const T k2 = n_dist > 1 ? T(get_dist(1)) : T(0);
    const T p1 = n_dist > 2 ? T(get_dist(2)) : T(0);
    const T p2 = n_dist > 3 ? T(get_dist(3)) : T(0);
    const T k3 = n_dist > 4 ? T(get_dist(4)) : T(0);

    norm_pts.clear();
    norm_pts.reserve(obj_pts.size());

    for (const auto& pt3: obj_pts) {
        Eigen::Matrix<T, 3, 1> Pw(T(pt3.x), T(pt3.y), T(pt3.z));
        Eigen::Matrix<T, 3, 1> Pc = R * Pw + t;

        T Xc = Pc(0);
        T Yc = Pc(1);
        T Zc = Pc(2);

        T x = Xc / Zc;
        T y = Yc / Zc;

        T r2 = x * x + y * y;
        T r4 = r2 * r2;
        T r6 = r4 * r2;

        T radial = T(1) + k1 * r2 + k2 * r4 + k3 * r6;

        T xd = x * radial + T(2) * p1 * x * y + p2 * (r2 + T(2) * x * x);
        T yd = y * radial + p1 * (r2 + T(2) * y * y) + T(2) * p2 * x * y;

        norm_pts.emplace_back(xd, yd);
    }
}
[[nodiscard]] inline double lerp_angle(double a0, double a1, double t) noexcept {
    double d = std::remainder(a1 - a0, 2.0 * M_PI);
    return a0 + t * d;
}
[[nodiscard]] inline Vec3 load_vec3(const YAML::Node& node) {
    auto vec = node.as<std::vector<double>>();
    return Vec3(vec[0], vec[1], vec[2]);
}
[[nodiscard]] inline Mat3 load_mat3(const YAML::Node& node) {
    Mat3 result;

    if (node.IsSequence() && node.size() == 9) {
        // 一维数组
        auto vec = node.as<std::vector<double>>();
        for (int i = 0; i < 9; ++i) {
            result(i / 3, i % 3) = vec[i];
        }
    } else {
        // 二维数组
        auto mat = node.as<std::vector<std::vector<double>>>();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result(i, j) = mat[i][j];
            }
        }
    }

    return result;
}
[[nodiscard]] inline ISO3 load_isometry3(const YAML::Node& node) {
    auto trans = load_vec3(node["t"]);
    auto rot = load_mat3(node["R"]);
    ISO3 result = ISO3::Identity();
    result.translation() = trans;
    result.linear() = rot;
    return result;
}
[[nodiscard]] inline cv::Rect2f load_rect2f(const YAML::Node& node) {
    auto x = node["x"].as<double>();
    auto y = node["y"].as<double>();
    auto w = node["w"].as<double>();
    auto h = node["h"].as<double>();
    return cv::Rect2f(x, y, w, h);
}
inline std::optional<std::string> get_arg(int i, int argc, char* argv[]) {
    if (i < argc) {
        std::cout << "get args " << std::string(argv[i]) << std::endl;
        return std::make_optional(std::string(argv[i]));
    }
    return std::nullopt;
};
template<class LandMark, class ObjectPoint>
inline ISO3 solve_pnp(
    const LandMark& landmarks,
    const ObjectPoint& object_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coefficients,
    int flags = cv::SOLVEPNP_ITERATIVE
) {
    cv::Mat rvec, tvec;
    cv::solvePnP(
        object_points,
        landmarks,
        camera_matrix,
        distortion_coefficients,
        rvec,
        tvec,
        false,
        flags
    );
    return rvec_tvec_to_eigen_iso3(rvec, tvec);
}
inline auto
calculate_distance_to_img_center(const cv::Point2f& image_point, const cv::Mat& camera_matrix) {
    auto cx = camera_matrix.at<double>(0, 2);
    auto cy = camera_matrix.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}
template<class Mat>
inline void
fill_constant_accel_noise(Mat& q, int pos_idx, int vel_idx, double noise, double dt) noexcept {
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    q(pos_idx, pos_idx) = dt4 * 0.25 * noise;
    q(pos_idx, vel_idx) = dt3 * 0.5 * noise;
    q(vel_idx, pos_idx) = dt3 * 0.5 * noise;
    q(vel_idx, vel_idx) = dt2 * noise;
}
[[nodiscard]] inline double sigmoid(double x) noexcept {
    return x >= 0 ? 1.0 / (1.0 + std::exp(-x)) : std::exp(x) / (1.0 + std::exp(x));
}

[[nodiscard]] inline float rect_ioU(const cv::Rect2f& a, const cv::Rect2f& b) noexcept {
    const cv::Rect2f inter = a & b;
    const float inter_area = inter.area();
    const float union_area = a.area() + b.area() - inter_area;
    if (union_area <= 0.f || std::isnan(union_area))
        return 0.f;
    return inter_area / union_area;
}
template<typename Func, typename T>
[[nodiscard]] inline T
golden_section_search(Func&& f, T left, T right, T eps = static_cast<T>(1e-4)) {
    static_assert(std::is_floating_point_v<T>);

    constexpr T phi = static_cast<T>(0.6180339887498948482);

    T x1 = right - phi * (right - left);
    T x2 = left + phi * (right - left);

    T f1 = f(x1);
    T f2 = f(x2);

    while ((right - left) > eps) {
        if (f1 > f2) {
            left = x1;

            x1 = x2;
            f1 = f2;

            x2 = left + phi * (right - left);
            f2 = f(x2);
        } else {
            right = x2;

            x2 = x1;
            f2 = f1;

            x1 = right - phi * (right - left);
            f1 = f(x1);
        }
    }

    return (f1 < f2) ? x1 : x2;
}
template<typename RectType>
inline RectType
expand_and_clip_rect(const RectType& rect, double expand_ratio, const cv::Size& img_size) {
    RectType r = rect;
    r.x -= (r.width * (expand_ratio - 1.0) * 0.5);
    r.y -= (r.height * (expand_ratio - 1.0) * 0.5);
    r.width = (r.width * expand_ratio);
    r.height = (r.height * expand_ratio);
    RectType img_rect(0, 0, img_size.width, img_size.height);
    r = r & img_rect;
    return r;
}

template<typename T>
bool is_finite(const T& s) {
    return std::apply([](auto... v) { return (std::isfinite(v) && ...); }, s.to_tuple());
}
} // namespace awakening::utils
