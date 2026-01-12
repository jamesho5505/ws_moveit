import cv2
import pyrealsense2 as rs
import numpy as np
from scipy.spatial.transform import Rotation as R



def fit_circle_least_squares(points_xy):
    x_coords = points_xy[:, 0]
    y_coords = points_xy[:, 1]

    A_matrix = np.column_stack([
        2 * x_coords,
        2 * y_coords,
        np.ones(len(points_xy))
    ])
    b_vector = x_coords**2 + y_coords**2

    c_params, _, _, _ = np.linalg.lstsq(
        A_matrix,
        b_vector,
        rcond=None
    )

    center_x = c_params[0]
    center_y = c_params[1]
    radius = np.sqrt(
        c_params[2] + center_x**2 + center_y**2
    )

    return int(center_x), int(center_y), int(radius)

# def pixel_to_camera_xyz(
#     depth_frame,
#     intrinsics,
#     u_px,
#     v_px
# ):
#     depth_m = depth_frame.get_distance(u_px, v_px)
#     if depth_m <= 0.0:
#         return None

#     X = (u_px - intrinsics.ppx) * depth_m / intrinsics.fx
#     Y = (v_px - intrinsics.ppy) * depth_m / intrinsics.fy
#     Z = depth_m
#     return np.array([X, Y, Z])

def robust_pixel_to_camera_xyz(
    depth_frame,
    intrinsics,
    u_px,
    v_px,
    window=3
):
    depths = []
    for du in range(-window, window + 1):
        for dv in range(-window, window + 1):
            uu = u_px + du
            vv = v_px + dv
            d = depth_frame.get_distance(uu, vv)
            if d > 0:
                depths.append(d)

    if len(depths) < 5:
        return None

    depth_m = np.median(depths)

    X = (u_px - intrinsics.ppx) * depth_m / intrinsics.fx
    Y = (v_px - intrinsics.ppy) * depth_m / intrinsics.fy
    Z = depth_m
    return np.array([X, Y, Z])


def fit_plane_svd(points_xyz):
    centroid = np.mean(points_xyz, axis=0)
    centered = points_xyz - centroid
    _, _, vh = np.linalg.svd(centered)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    return centroid, normal

def build_plane_axes(normal_vector):
    z_axis = normal_vector
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(tmp, z_axis)) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])

    x_axis = np.cross(tmp, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    return x_axis, y_axis, z_axis

def generate_circle_3d(
    center_xyz,
    axis_x,
    axis_y,
    radius_m,
    num_points
):
    points = []
    for i in range(num_points):
        theta = 2.0 * np.pi * i / num_points
        p = (
            center_xyz
            + radius_m * np.cos(theta) * axis_x
            + radius_m * np.sin(theta) * axis_y
        )
        points.append(p)
    return np.array(points)

def pose6d_to_T_base_flange(pose6d):
    T = np.eye(4)
    T[:3, 3] = pose6d[:3]          # m
    R_mat = R.from_euler('zyx', pose6d[3:], degrees=False).as_matrix()
    T[:3, :3] = R_mat
    return T



def capture_realsense_and_binarize():
    # === RealSense pipeline 設定 ===
    realsense_pipeline = rs.pipeline()
    realsense_config = rs.config()

    realsense_config.enable_stream(
        rs.stream.color,
        1280,
        720,
        rs.format.bgr8,
        30
    )

    realsense_config.enable_stream(
        rs.stream.depth,
        1280,
        720,
        rs.format.z16,
        30
    )


    realsense_pipeline.start(realsense_config)
    print("按 s 拍照並二值化，按 q 離開")

    try:
        while True:
            frames = realsense_pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            depth_frame = frames.get_depth_frame()
            
            if not depth_frame:
                continue

            profile = realsense_pipeline.get_active_profile()
            depth_stream = profile.get_stream(rs.stream.depth)
            depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()


            color_image = np.asanyarray(color_frame.get_data())

            # === 灰階 ===
            grayscale_image = cv2.cvtColor(
                color_image,
                cv2.COLOR_BGR2GRAY
            )

            # === 二值化（Otsu） ===
            binary_image = cv2.adaptiveThreshold(
                grayscale_image,
                maxValue=255,
                adaptiveMethod=cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                thresholdType=cv2.THRESH_BINARY,
                blockSize=11,
                C=2
            )

            # === 二值化（Otsu） ===
            # otsu_threshold_value, binary_image = cv2.threshold(
            #     grayscale_image,
            #     10,
            #     255,
            #     cv2.THRESH_BINARY + cv2.THRESH_OTSU
            # )

            # === 尋找邊緣 ===

            # blurred_image = cv2.GaussianBlur(
            #     grayscale_image,
            #     ksize=(7, 7),   # 可試 (7,7)
            #     sigmaX=1.5
            # )

            blurred_image = cv2.medianBlur(
                grayscale_image,
                ksize=7
            )

            # clahe = cv2.createCLAHE(
            #     clipLimit=2.0,
            #     tileGridSize=(8, 8)
            # )
            # contrast_image = clahe.apply(blurred_image)

            # canny_image = cv2.Canny(
            #     contrast_image,
            #     threshold1=50,
            #     threshold2=150
            # )

            canny_image = cv2.Canny(
                blurred_image,
                threshold1=50,
                threshold2=150
            )

            # kernel_for_dilation = np.ones((5, 5), np.uint8)
            # canny_image = cv2.dilate(
            #     canny_image,
            #     kernel_for_dilation,
            #     iterations=6
            # )

            # contours, hierarchy = cv2.findContours(
            #     canny_image,
            #     cv2.RETR_TREE,
            #     cv2.CHAIN_APPROX_SIMPLE
            # )

            contours, hierarchy = cv2.findContours(
                canny_image,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_NONE
            )

            largest_contour = max(
                contours,
                key=cv2.contourArea
            )

            (rough_center_x, rough_center_y), rough_radius = cv2.minEnclosingCircle(
                largest_contour
            )

            rough_center_x = int(rough_center_x)
            rough_center_y = int(rough_center_y)
            rough_radius = int(rough_radius)

            roi_margin_pixels = 20

            roi_x_min = max(rough_center_x - rough_radius - roi_margin_pixels, 0)
            roi_x_max = min(rough_center_x + rough_radius + roi_margin_pixels, canny_image.shape[1])
            roi_y_min = max(rough_center_y - rough_radius - roi_margin_pixels, 0)
            roi_y_max = min(rough_center_y + rough_radius + roi_margin_pixels, canny_image.shape[0])

            roi_edges = canny_image[roi_y_min:roi_y_max, roi_x_min:roi_x_max]


            roi_contours, _ = cv2.findContours(
                roi_edges,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_NONE
            )

            roi_largest_contour = max(
                roi_contours,
                key=cv2.contourArea
            )

            roi_largest_contour = roi_largest_contour + np.array([[roi_x_min, roi_y_min]])
            circle_points = roi_largest_contour.reshape(-1, 2)
            final_center_x, final_center_y, final_radius = fit_circle_least_squares(circle_points)


            circle_visualization = color_image.copy()
            cv2.circle(
                circle_visualization,
                (final_center_x, final_center_y),
                final_radius,
                (0, 255, 0),
                2
            )

            cv2.circle(
                circle_visualization,
                (final_center_x, final_center_y),
                4,
                (0, 0, 255),
                -1
            )

            drawing = np.zeros((canny_image.shape[0], canny_image.shape[1], 3), dtype=np.uint8)
            
            for i in range(len(contours)):
                color = (255, 255, 255)
                cv2.drawContours(drawing, contours, i, color, 2, cv2.LINE_8, hierarchy, 0)
                # cv2.drawContours(circle_visualization, contours, i, (0, 255, 0), 2, cv2.LINE_8, hierarchy, 0)


            # if detected_circles is not None:
            #     detected_circles = np.uint16(np.around(detected_circles))

            #     for circle_x, circle_y, circle_radius in detected_circles[0]:
            #         cv2.circle(
            #             circle_visualization,
            #             (circle_x, circle_y),
            #             circle_radius,
            #             (0, 255, 0),
            #             2
            #         )
            #         cv2.circle(
            #             circle_visualization,
            #             (circle_x, circle_y),
            #             5,
            #             (0, 0, 255),
            #             -1
            #         )


            circle_points_camera = []

            for (u, v) in circle_points:
                u_i, v_i = int(u), int(v)
                p_cam = robust_pixel_to_camera_xyz(
                    depth_frame,
                    depth_intrinsics,
                    u_i,
                    v_i
                )
                if p_cam is not None:
                    circle_points_camera.append(p_cam)

            circle_points_camera = np.array(circle_points_camera)
            plane_center_camera, plane_normal_camera = fit_plane_svd(circle_points_camera)
            # 確保法向量朝向相機（Z > 0）
            if plane_normal_camera[2] > 0:
                plane_normal_camera = -plane_normal_camera

            axis_x, axis_y, axis_z = build_plane_axes(plane_normal_camera)



            circle_radius_m = np.mean(
                np.linalg.norm(circle_points_camera - plane_center_camera, axis=1)
            )

            circle_points_camera_3d = generate_circle_3d(
                plane_center_camera,
                axis_x,
                axis_y,
                circle_radius_m,
                num_points=120
            )

            circle_points_base = []

            T_flange_camera_optical= np.array([
                [-0.99926981,  0.03763377, -0.00659936,  0.0342429 ],
                [-0.03773388, -0.99916346,  0.01576506, -0.08719725],
                [-0.00600054,  0.01600256,  0.99985395,  0.06636586],
                [ 0.,          0.,          0.,          1.]
            ])

            T_base_flange_at_capture = np.array([
                [-1.00000000e+00, -1.03863282e-05,  2.22579166e-05,  1.49798203e-01],
                [-1.03863283e-05,  1.00000000e+00, -1.22464680e-16, -3.06440399e-01],
                [-2.22579166e-05, -2.31178150e-10, -1.00000000e+00,  3.08154266e-01],
                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
            ])

            for p_cam in circle_points_camera_3d:
                p_cam_h = np.append(p_cam, 1.0)
                p_flange = T_flange_camera_optical @ p_cam_h
                p_base   = T_base_flange_at_capture @ p_flange
                circle_points_base.append(p_base[:3])

            circle_points_base = np.array(circle_points_base)

            

            cv2.imshow("Color Image", color_image)
            cv2.imshow("Grayscale Image", grayscale_image)
            cv2.imshow("Binary Image (Otsu)", binary_image)
            cv2.imshow("Contours", drawing)
            cv2.imshow("Detected Circle", circle_visualization)


            key_pressed = cv2.waitKey(1) & 0xFF

            if key_pressed == ord('s'):
                cv2.imwrite("realsense_color.jpg", color_image)
                cv2.imwrite("realsense_grayscale.jpg", grayscale_image)
                cv2.imwrite("realsense_binary_otsu.jpg", binary_image)
                cv2.imwrite("realsense_contours.jpg", drawing)
                cv2.imwrite("realsense_detected_circle.jpg", circle_visualization)
                print("已儲存影像檔案")

            elif key_pressed == ord('q'):
                break

            elif key_pressed == ord('c'):
                print("圓心座標 (像素):", final_center_x, final_center_y)
                print("圓半徑 (像素):", final_radius)
                print("圓半徑 (公尺):", circle_radius_m)
                print("平面法向量 (相機座標):", plane_normal_camera)
                print("平面中心點 (相機座標):", plane_center_camera)

    finally:
        realsense_pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    capture_realsense_and_binarize()
