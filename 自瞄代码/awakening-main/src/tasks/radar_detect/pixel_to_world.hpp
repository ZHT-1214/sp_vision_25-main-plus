#pragma once
#include <open3d/Open3D.h>
#include <open3d/core/Tensor.h>
#include <open3d/io/TriangleMeshIO.h>
#include <open3d/t/geometry/RaycastingScene.h>
#include <opencv2/core/types.hpp>
#include <optional>
#include <yaml-cpp/node/node.h>
namespace awakening::radar_detect {
class PixelToWorld {
public:
    PixelToWorld(
        const YAML::Node& config,
        const ISO3& camera_cv_in_target_map,
        const CameraInfo& camera_info
    ) {
        camera_cv_in_target_map_ = camera_cv_in_target_map;
        camera_info_ = camera_info;
        auto mesh_path = config["mesh_path"].as<std::string>();
        mesh_ = open3d::io::CreateMeshFromFile(mesh_path);
        if (!mesh_) {
            throw std::runtime_error("Failed to create mesh from file");
        }
        scene_ = std::make_shared<open3d::t::geometry::TriangleMesh>(
            open3d::t::geometry::TriangleMesh::FromLegacy(*mesh_)
        );

        scene3d_ = std::make_shared<open3d::t::geometry::TriangleMesh>(scene_->Clone());

        raycasting_scene_ = std::make_shared<open3d::t::geometry::RaycastingScene>();
        raycasting_scene_->AddTriangles(*scene_);

        AWAKENING_INFO("Mesh加载成功, 顶点数: {}", mesh_->vertices_.size());
    }
    std::optional<Eigen::Vector3d> pixel_to_world(const cv::Point2f& pixel) {
        double fx = camera_info_.camera_matrix.at<double>(0, 0);
        double fy = camera_info_.camera_matrix.at<double>(1, 1);
        double cx = camera_info_.camera_matrix.at<double>(0, 2);
        double cy = camera_info_.camera_matrix.at<double>(1, 2);

        float x = (pixel.x - cx) / fx;
        float y = (pixel.y - cy) / fy;

        Eigen::Vector3f ray_direction(x, y, 1.0);
        ray_direction.normalize(); // 单位化射线方向
        ray_direction = camera_cv_in_target_map_.linear().cast<float>() * ray_direction;
        Eigen::Vector3f ray_origin = camera_cv_in_target_map_.translation().cast<float>();

        std::vector<float> ray_data = { ray_origin.x(),    ray_origin.y(),    ray_origin.z(),
                                        ray_direction.x(), ray_direction.y(), ray_direction.z() };

        open3d::core::Tensor rays_tensor(
            ray_data.data(),
            { 1, 6 },
            open3d::core::Dtype::Float32
        ); // {1, 6} shape means 1 ray with origin (3) and direction (3)

        auto raycast_result = raycasting_scene_->CastRays(rays_tensor);

        auto t_hit = raycast_result["t_hit"];
        if (t_hit.NumElements() == 0) {
            return std::nullopt;
        }

        auto t_hit_vec = t_hit.ToFlatVector<float>();

        if (t_hit_vec.empty()) {
            return std::nullopt;
        }

        auto t = t_hit_vec[0];

        Eigen::Vector3f pt_intersect = ray_origin + t * ray_direction;
        return pt_intersect.cast<double>();
    }
    ISO3 camera_cv_in_target_map_;
    CameraInfo camera_info_;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh_;
    std::shared_ptr<open3d::t::geometry::TriangleMesh> scene_;
    std::shared_ptr<open3d::t::geometry::TriangleMesh> scene3d_;
    std::shared_ptr<open3d::t::geometry::RaycastingScene> raycasting_scene_;
};
} // namespace awakening::radar_detect