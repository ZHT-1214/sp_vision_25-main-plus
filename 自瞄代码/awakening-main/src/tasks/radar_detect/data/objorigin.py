import open3d as o3d
import numpy as np

mesh = o3d.io.read_triangle_mesh("/home/hy/awakening/radar.stl")

if mesh.is_empty():
    print("加载失败")
    exit(1)

# 转成点云（更容易选）
pcd = mesh.sample_points_uniformly(number_of_points=500000)

print("Shift + 左键 选点，Q 退出")

vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(pcd)
vis.run()
vis.destroy_window()

picked = vis.get_picked_points()

if len(picked) == 0:
    print("没选中点")
    exit(1)

points = np.asarray(pcd.points)
new_origin = points[picked[0]]

print("选中的点:", new_origin)

# 平移 mesh（不是 pcd）
mesh.translate(-new_origin)

mesh.compute_vertex_normals()

# 可视化
o3d.visualization.draw_geometries([mesh])

# 保存 STL
o3d.io.write_triangle_mesh("out.stl", mesh)