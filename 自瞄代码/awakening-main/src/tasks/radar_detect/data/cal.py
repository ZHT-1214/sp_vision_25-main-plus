import cv2
import yaml
import numpy as np

# =============================
# YAML 文件路径
# =============================
YAML_PATH = "/home/hy/radar_data/feild_point.yaml"
IMAGE_PATH = "/home/hy/awakening/guanggu.png"

# =============================
# 读取 YAML
# =============================
with open(YAML_PATH, "r") as f:
    data = yaml.safe_load(f)

pts3d = np.array(data["pts3d"], dtype=np.float32)
camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)

# =============================
# 加载图像
# =============================
img = cv2.imread(IMAGE_PATH)
if img is None:
    raise RuntimeError("图像读取失败")

display = img.copy()
pts2d = []

# =============================
# 重绘函数
# =============================
def redraw_display():
    global display
    display = img.copy()
    for idx, (x, y) in enumerate(pts2d):
        cv2.circle(display, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(display, str(idx), (x+10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("Image", display)

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pts2d.append([x, y])
        redraw_display()
        print(f"Point {len(pts2d)-1}: ({x}, {y})")

cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Image", 1280, 720)
cv2.setMouseCallback("Image", mouse_callback)

cv2.namedWindow("Reprojection", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Reprojection", 1280, 720)

print(f"请按顺序点击 {len(pts3d)} 个点，Enter运行PnP，ESC退出")
print("按 'u' 撤回最后一个点")
print("方向键可微调最后一个点像素位置（上下左右）")

# =============================
# 主循环
# =============================
while True:
    cv2.imshow("Image", display)
    key = cv2.waitKey(10) & 0xFF

    if key == 27:  # ESC 退出
        break

    # 撤回最后一个点
    if key == ord('u'):
        if pts2d:
            removed = pts2d.pop()
            print(f"撤回点: {removed}")
            redraw_display()
        continue

    # 微调最后一个点
    if pts2d:
        dx, dy = 0, 0
        if key == 81:   # 左
            dx = -1
        elif key == 82: # 上
            dy = -1
        elif key == 83: # 右
            dx = 1
        elif key == 84: # 下
            dy = 1
        if dx != 0 or dy != 0:
            pts2d[-1][0] += dx
            pts2d[-1][1] += dy
            redraw_display()
            print(f"微调最后一个点: {pts2d[-1]}")

    # 运行PnP
    if key == 13:  # Enter 键
        if len(pts2d) < 4:
            print("至少需要4个点")
            continue

        pts2d_np = np.array(pts2d, dtype=np.float32)

        # =============================
        # 顺序对应 PnP
        # =============================
        success, rvec, tvec = cv2.solvePnP(
            pts3d[:len(pts2d_np)], pts2d_np,
            camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            print("PnP失败")
            continue

        # 世界->相机
        R_wc, _ = cv2.Rodrigues(rvec)
        t_wc = tvec

        # 相机->世界
        R_cw = R_wc.T
        t_cw = -R_wc.T @ t_wc

        print("\n===== PnP结果 =====")
        print("rvec:\n", rvec)
        print("tvec:\n", tvec)
        print("R_cw:\n", R_cw)
        print("t_cw:\n", t_cw)

        # 保存 YAML
        yaml_data = {
            "t": t_cw.flatten().tolist(),
            "R": R_cw.flatten().tolist(),
            "cal_pts": [list(p) for p in pts2d_np.astype(int)]
        }
        yaml_path = "PnP_result.yaml"
        with open(yaml_path, "w") as f:
            f.write(f"t: {yaml_data['t']}\n")
            f.write(f"R: {yaml_data['R']}\n")
            f.write("cal_pts:\n")
            for p in yaml_data['cal_pts']:
                f.write(f"  - {p}\n")

        print(f"✅ YAML 保存完成: {yaml_path}")

        # 重投影可视化
        reproj, _ = cv2.projectPoints(
            pts3d[:len(pts2d_np)], rvec, tvec,
            camera_matrix, dist_coeffs
        )
        reproj = reproj.reshape(-1, 2)

        vis = img.copy()
        for p1, p2 in zip(pts2d_np, reproj):
            cv2.circle(vis, tuple(p1.astype(int)), 5, (0, 255, 0), -1)
            cv2.circle(vis, tuple(p2.astype(int)), 5, (0, 0, 255), -1)
            cv2.line(vis, tuple(p1.astype(int)), tuple(p2.astype(int)), (255, 255, 0), 2)

        cv2.imshow("Reprojection", vis)
        cv2.imwrite("PnP_result.png", vis)
        print("✅ 重投影可视化保存完成: PnP_result.png")

cv2.destroyAllWindows()