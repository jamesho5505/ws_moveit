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


            cv2.imshow("Color Image", color_image)
            cv2.imshow("Grayscale Image", grayscale_image)
            cv2.imshow("Binary Image (Otsu)", binary_image)

            key_pressed = cv2.waitKey(1) & 0xFF

            if key_pressed == ord('s'):
                cv2.imwrite("realsense_color.jpg", color_image)
                cv2.imwrite("realsense_grayscale.jpg", grayscale_image)
                cv2.imwrite("realsense_binary_otsu.jpg", binary_image)

            elif key_pressed == ord('q'):
                break

    finally:
        realsense_pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    capture_realsense_and_binarize()
