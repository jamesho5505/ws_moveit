#!/usr/bin/env python3
# correction_hole_calibrated.py — use fixed T_{A<-C} to output hole pose in robot base
import os, math, numpy as np, cv2, torch
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

def clamp(v, lo, hi): return max(lo, min(hi, v))

def results_xyxy_list(res):
    out=[]; xyxy=None
    try:
        if hasattr(res,'obb') and res.obb is not None and getattr(res.obb,'xyxy',None) is not None:
            xyxy = res.obb.xyxy
        elif hasattr(res,'boxes') and res.boxes is not None and getattr(res.boxes,'xyxy',None) is not None:
            xyxy = res.boxes.xyxy
    except: xyxy=None
    if xyxy is None: return out
    xyxy = xyxy.cpu().numpy() if hasattr(xyxy,'cpu') else np.asarray(xyxy)
    for row in xyxy:
        x1,y1,x2,y2 = map(float,row[:4])
        if x2>x1 and y2>y1:
            out.append((x1,y1,x2,y2,(x2-x1)*(y2-y1)))
    return out

class HoleCalibSingleFrame(Node):
    def __init__(self):
        super().__init__('hole_calib_single_frame'); self.bridge = CvBridge()

        # --- topics/params ---
        self.color_topic = self.declare_parameter('color_topic','/camera/camera/color/image_raw').value
        self.depth_topic = self.declare_parameter('depth_topic','/camera/camera/aligned_depth_to_color/image_raw').value
        self.info_topic  = self.declare_parameter('camera_info_topic','/camera/camera/color/camera_info').value
        self.stabilization_frames = int(self.declare_parameter('stabilization_frames', 10).value)

        # ROI
        self.roi_width_ratio  = float(self.declare_parameter('roi_width_ratio', 0.25).value)
        self.roi_height_ratio = float(self.declare_parameter('roi_height_ratio', 0.25).value)
        self.roi_left_margin  = int(self.declare_parameter('roi_left_margin', 60).value)
        self.roi_bottom_margin= int(self.declare_parameter('roi_bottom_margin', 60).value)
        self.dot_r = int(self.declare_parameter('dot_radius', 2).value)

        # optional base offset (m)
        self.declare_parameter('position_correction_x', 0.0)
        self.declare_parameter('position_correction_y', 0.0)
        self.declare_parameter('position_correction_z', 0.0)
        self.position_correction = np.array([
            self.get_parameter('position_correction_x').value,
            self.get_parameter('position_correction_y').value,
            self.get_parameter('position_correction_z').value
        ], dtype=np.float64)

        # === FIXED calibration T_{A<-C} (units must match your data; here: meters) ===
        self.T_A_C = np.array(
            [


            # [ 1.00578236, -0.01811072, -0.00865681,  0.09088098],
            # [-0.01388196, -1.00113625,  0.00319743,  0.43842693],
            # [ 0.00539054, -0.00862081, -0.99074636, 0.42509617],
            # [ 0.,          0.,          0.,          1.        ]

            # [ 1.00595138, -0.01862445,-0.00737135,  0.09046828],
            # [-0.01382536, -1.00141779,  0.00283774,  0.43851448],
            # [ 0.00280344, -0.0084262,  -0.99693067,  0.4270352 ],
            # [ 0.,          0.,          0.,          1.        ]

            # [ 1.00613151, -0.01859431, -0.00712982,  0.09039068],
            # [-0.01389467, -1.00147827,  0.00311466,  0.43842151],
            # [ 0.00218016, -0.00799811, -0.99691601,  0.42706775],
            # [ 0.,          0.,          0.,          1.        ]

            # [ 0.99625327, -0.02677146, -0.00585344,  0.08988328],
            # [-0.02275659, -1.00427307,  0.00423345,  0.43782561],
            # [ 0.00582773, -0.00945535, -1.00369201,  0.42827939],
            # [ 0.,          0.,          0.,          1.        ]

            # [ 0.99625315, -0.02671292, -0.00598979,  0.0899306 ],
            # [-0.02275697, -1.00409045,  0.00380811,  0.43797323],
            # [ 0.00582779, -0.0094855,  -1.00362179,  0.42825502],
            # [ 0.,          0.,          0.,          1.        ]

            [ 0.9962597,  -0.02670584, -0.00599904,  0.08993219],
            [-0.0227818,  -1.00411728,  0.00384315,  0.4379672 ],
            [ 0.00583895, -0.00947345, -1.00363753,  0.42825773],
            [ 0.,          0.,          0.,          1.        ],

        ], dtype=np.float64)





        # YOLO
        share_dir  = get_package_share_directory('detector')
        model_path = os.path.join(share_dir,'models', self.declare_parameter('yolo_model_name','block_obb.pt').value)
        self.yolo_conf = float(self.declare_parameter('yolo_conf',0.25).value)
        self.yolo_iou  = float(self.declare_parameter('yolo_iou',0.50).value)
        self.pick_strategy = self.declare_parameter('pick_strategy','largest').value
        self.model = YOLO(model_path).to('cuda' if torch.cuda.is_available() else 'cpu')

        # intrinsics + depth cache
        self.fx=self.fy=self.cx=self.cy=None
        self.last_depth=None; self.depth_encoding=None

        # state
        self.frame_counter=0; self.stabilized_frame=None; self.stabilized_depth=None; self.processing_done=False

        # GUI
        cv2.namedWindow('Frame 1 - Stabilization', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Frame 2 - YOLO Detection', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Frame 3 - Hole Detection', cv2.WINDOW_AUTOSIZE)
        # cv2.namedWindow('Frame 4A', cv2.WINDOW_AUTOSIZE)

        # subs
        self.create_subscription(CameraInfo,self.info_topic,self.cb_info,10)
        self.create_subscription(Image,self.color_topic,self.cb_color,10)
        self.create_subscription(Image,self.depth_topic,self.cb_depth,10)
        self.get_logger().info(f"Initialized. Waiting {self.stabilization_frames} frames...")

    # ---------- Callbacks ----------
    def cb_info(self, msg: CameraInfo):
        K=msg.k; self.fx,self.fy,self.cx,self.cy = K[0],K[4],K[2],K[5]
    def cb_depth(self, msg: Image):
        self.depth_encoding = msg.encoding
        self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # ---------- tools ----------
    def calculate_roi_bottom_left(self, bbox, img_shape):
        h, w = img_shape; x1,y1,x2,y2=bbox; bw,bh=(x2-x1),(y2-y1)
        roi_w=bw*self.roi_width_ratio; roi_h=bh*self.roi_height_ratio
        rx=x1+self.roi_left_margin; ry=y2-self.roi_bottom_margin-roi_h
        rx0=int(clamp(rx,0,w-1)); ry0=int(clamp(ry,0,h-1))
        rx1=int(clamp(rx+roi_w,0,w-1)); ry1=int(clamp(ry+roi_h,0,h-1))
        return rx0,ry0,rx1,ry1

    def pixel_to_camera_coords(self, u, v, depth_mm):
        # output in meters
        if self.fx is None or depth_mm is None: return None
        x = (u - self.cx) * depth_mm / self.fx / 1000.0
        y = (v - self.cy) * depth_mm / self.fy / 1000.0
        z = depth_mm / 1000.0
        return np.array([x,y,z], dtype=np.float64)

    def cam_to_base(self, Xc):
        # Xc [m] in camera -> Xa [m] in base using fixed calibration
        Xh = np.array([Xc[0], Xc[1], Xc[2], 1.0], dtype=np.float64)
        v = (self.T_A_C @ Xh)[:3]
        return v + self.position_correction

    def get_depth_mm_from_image(self, r, c, depth_img):
        if depth_img is None: return None
        h,w = depth_img.shape[:2]
        if r<0 or r>=h or c<0 or c>=w: return None
        d = depth_img[r, c]; enc=str(self.depth_encoding)
        if ('16UC1' in enc) or (enc in ('16UC1','mono16')): z_mm = float(d)
        elif ('32FC1' in enc) or (enc=='32FC1'):            z_mm = float(d) * 1000.0
        else:
            z = float(d); z_mm = z if z>50 else z*1000.0
        if not math.isfinite(z_mm) or z_mm<=0 or z_mm>6000: return None
        return z_mm

    # depth từ vành + lọc outlier: bỏ min/max và bỏ giá trị > median+tol
    def depth_from_ring_filtered(self, contour, depth_img,
                                 ring_w=5, min_samples=30,
                                 tol_far_mm=5.0, drop_minmax=True):
        if depth_img is None: return None
        h,w = depth_img.shape[:2]
        mask = np.zeros((h, w), np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, 1)
        ring = cv2.dilate(mask, np.ones((ring_w, ring_w), np.uint8), 1)
        ring = cv2.bitwise_or(ring, mask)
        ys, xs = np.where(ring > 0)

        vals=[]
        for rr, cc in zip(ys, xs):
            z = self.get_depth_mm_from_image(rr, cc, depth_img)
            if z is not None: vals.append(z)
        if len(vals) < min_samples: return None

        a = np.asarray(vals, dtype=np.float64)
        if drop_minmax and a.size >= 3: a = np.sort(a)[1:-1]
        med = np.median(a)
        a = a[a <= (med + tol_far_mm)]
        if a.size < max(5, min_samples//3): return None
        return float(np.median(a))

    # ---------- main ----------
    def process_single_frame(self):
        if self.stabilized_frame is None or self.stabilized_depth is None:
            self.get_logger().error("No stabilized frame"); return
        bgr=self.stabilized_frame.copy(); depth_img=self.stabilized_depth.copy()
        h,w=bgr.shape[:2]

        # === Frame 2: YOLO detection ===
        res = self.model.predict(source=bgr, imgsz=max(h,w), conf=self.yolo_conf, iou=self.yolo_iou, verbose=False)[0]
        boxes = results_xyxy_list(res)
        if not boxes:
            self.get_logger().error("YOLO found no object"); return
        if self.pick_strategy=='center':
            boxes.sort(key=lambda t: ((t[0]+t[2])/2 - w/2)**2 + ((t[1]+t[3])/2 - h/2)**2)
        else:
            boxes.sort(key=lambda t: t[4], reverse=True)
        x1,y1,x2,y2 = boxes[0][:4]
        yolo_vis=bgr.copy()
        cv2.rectangle(yolo_vis,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),2)
        rx0,ry0,rx1,ry1 = self.calculate_roi_bottom_left((x1,y1,x2,y2),(h,w))
        cv2.rectangle(yolo_vis,(rx0,ry0),(rx1,ry1),(0,0,255),2)
        cv2.imshow('Frame 2 - YOLO Detection', yolo_vis)

        # === Frame 3: Hole detection ===
        gray=cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray=cv2.createCLAHE(2.0,(8,8)).apply(gray)
        gray=cv2.bilateralFilter(gray, d=7, sigmaColor=30, sigmaSpace=7)
        roi_mask=np.zeros((h,w),np.uint8); cv2.rectangle(roi_mask,(rx0,ry0),(rx1,ry1),255,-1)

        def detect_hole(img, mask, mode):
            if mode=='otsu':      _,bw=cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            elif mode=='otsu_inv':_,bw=cv2.threshold(img,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
            else: bw=cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,31,2)
            bw=cv2.bitwise_and(bw,mask)
            cnts,_=cv2.findContours(bw,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            best=(0.0,0.0,None,None)
            for c in cnts:
                area=cv2.contourArea(c)
                if area<20: continue
                peri=cv2.arcLength(c,True)
                if peri<=1e-6: continue
                circ=4*math.pi*area/(peri*peri)
                if circ>best[0] or (abs(circ-best[0])<1e-3 and area>best[1]):
                    m=cv2.moments(c)
                    if m['m00']>1e-6:
                        cx=m['m10']/m['m00']; cy=m['m01']/m['m00']
                        best=(circ,area,(cx,cy),c)
            return bw,best

        candidates=[]
        for m in ['otsu','otsu_inv','adaptive']:
            bw,resu=detect_hole(gray,roi_mask,m); candidates.append((bw,resu))
        binary_img,(circularity,area,center,contour)=max(candidates,key=lambda x:(x[1][0],x[1][1]))
        if center is None:
            self.get_logger().error("No hole in ROI"); return

        u,v=center

        # ====== METHOD A: depth từ VÀNH + đổi sang base bằng T_{A<-C} ======
        depth_mm = self.depth_from_ring_filtered(
            contour=contour, depth_img=depth_img,
            ring_w=5, min_samples=30, tol_far_mm=5.0, drop_minmax=True
        )
        if depth_mm is None:
            self.get_logger().error("Depth invalid around ring; skip frame.")
            return

        cam_xyz  = self.pixel_to_camera_coords(u, v, depth_mm)  # [m] in {C}
        base_xyz = self.cam_to_base(cam_xyz)                     # [m] in {A}
        # ===================================================================

        # --- visualization ---
        hole_vis=cv2.cvtColor(binary_img,cv2.COLOR_GRAY2BGR)
        cv2.circle(hole_vis,(int(u),int(v)),self.dot_r,(0,0,255),-1)
        cv2.circle(hole_vis,(int(self.cx),int(self.cy)),max(2,self.dot_r),(0,255,0),-1)
        cv2.putText(hole_vis,"Hole Detection",(10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
        cv2.imshow('Frame 3 - Hole Detection', hole_vis)

        infoA=np.zeros((260,1000,3),dtype=np.uint8)
        linesA=[
            '=== Method A: Absolute ===',
            f'Pixel (u,v) [px]          = ({u:.1f}, {v:.1f})',
            f'Ring Depth Zc [mm]        = {depth_mm:.2f}',
            f'Camera XYZ [m]            = ({cam_xyz[0]:.9f}, {cam_xyz[1]:.9f}, {cam_xyz[2]:.6f})',
            f'Base hole XYZ [m]         = ({base_xyz[0]:.9f}, {base_xyz[1]:.9f}, {base_xyz[2]:.9f})'
        ]
        for i,s in enumerate(linesA):
            cv2.putText(infoA,s,(10,30+24*i),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),1,cv2.LINE_AA)
        cv2.imshow('Frame 4A', infoA)

        # log to console
        self.get_logger().info(f"cam_xyz [m]: {cam_xyz}")
        self.get_logger().info(f"base_xyz [m]: {base_xyz}")

        self.processing_done=True

    def cb_color(self, msg: Image):
        if self.fx is None: return
        if self.processing_done:
            key=cv2.waitKey(1)&0xFF
            if key==ord('q') or key==27:
                rclpy.shutdown()
            return
        bgr=self.bridge.imgmsg_to_cv2(msg,'bgr8')
        self.frame_counter+=1
        stab_vis=bgr.copy()
        cv2.putText(stab_vis, f"Stabilizing... {self.frame_counter}/{self.stabilization_frames}",
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
        cv2.imshow('Frame 1 - Stabilization', stab_vis)
        key=cv2.waitKey(1)&0xFF
        if key==ord('q') or key==27:
            rclpy.shutdown(); return
        if self.frame_counter==self.stabilization_frames and self.last_depth is not None:
            self.stabilized_frame=bgr.copy()
            self.stabilized_depth=self.last_depth.copy()
            self.get_logger().info("Stabilized. Processing...")
            self.process_single_frame()

def main():
    rclpy.init()
    node=HoleCalibSingleFrame()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows(); node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
