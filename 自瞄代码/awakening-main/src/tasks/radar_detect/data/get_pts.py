import open3d as o3d
import numpy as np

# 读取 STL mesh
mesh = o3d.io.read_triangle_mesh("/home/hy/awakening/src/tasks/radar_detect/data/radar.stl")

# 计算法线（可选）
mesh.compute_vertex_normals()

# 从 mesh 表面采样点云
pcd = mesh.sample_points_uniformly(
    number_of_points=5000000
)

# 显示点云
print("Shift + 左键 选择点")
print("Shift + 右键 取消选择")
print("按 Q 退出")

vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()

vis.add_geometry(pcd)

vis.run()
vis.destroy_window()

# 获取选择点索引
picked = vis.get_picked_points()

points = np.asarray(pcd.points)

print("\n选中的点：")

for idx in picked:
    p = points[idx]
    print(f"index={idx}")
    print(f"x={p[0]:.6f}")
    print(f"y={p[1]:.6f}")
    print(f"z={p[2]:.6f}")
    print()