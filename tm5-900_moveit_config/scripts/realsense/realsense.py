import cv2
import pyrealsense2 as rs
import numpy as np


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

    realsense_pipeline.start(realsense_config)
    print("按 s 拍照並二值化，按 q 離開")

    try:
        while True:
            frames = realsense_pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

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

            blurred_image = cv2.GaussianBlur(
                grayscale_image,
                ksize=(5, 5),   # 可試 (7,7)
                sigmaX=1.5
            )

            clahe = cv2.createCLAHE(
                clipLimit=2.0,
                tileGridSize=(8, 8)
            )
            contrast_image = clahe.apply(blurred_image)

            canny_image = cv2.Canny(
                contrast_image,
                threshold1=50,
                threshold2=150
            )

            # kernel_for_dilation = np.ones((3, 3), np.uint8)
            # canny_image = cv2.dilate(
            #     canny_image,
            #     kernel_for_dilation,
            #     iterations=6
            # )

            contours, hierarchy = cv2.findContours(
                canny_image,
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE
            )

            # contours, hierarchy = cv2.findContours(
            #     canny_image,
            #     cv2.RETR_EXTERNAL,
            #     cv2.CHAIN_APPROX_NONE
            # )

            drawing = np.zeros((canny_image.shape[0], canny_image.shape[1], 3), dtype=np.uint8)
            for i in range(len(contours)):
                color = (255, 255, 255)
                cv2.drawContours(drawing, contours, i, color, 2, cv2.LINE_8, hierarchy, 0)



            cv2.imshow("Color Image", color_image)
            cv2.imshow("Grayscale Image", grayscale_image)
            cv2.imshow("Binary Image (Otsu)", binary_image)
            cv2.imshow("Contours", drawing)


            key_pressed = cv2.waitKey(1) & 0xFF

            if key_pressed == ord('s'):
                cv2.imwrite("realsense_color.jpg", color_image)
                cv2.imwrite("realsense_grayscale.jpg", grayscale_image)
                cv2.imwrite("realsense_binary_otsu.jpg", binary_image)
                cv2.imwrite("realsense_contours.jpg", drawing)
                print("已儲存影像檔案")

            elif key_pressed == ord('q'):
                break

    finally:
        realsense_pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    capture_realsense_and_binarize()
