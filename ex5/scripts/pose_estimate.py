#!/usr/bin/env python3
import rclpy, cv2, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge

class ArucoView(Node):
    def __init__(self):
        super().__init__('aruco_view')
        # params
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_length_m', 0.15)

        it = self.get_parameter('image_topic').value
        ct = self.get_parameter('camera_info_topic').value
        dict_name = self.get_parameter('dictionary').value
        self.want_id = int(self.get_parameter('marker_id').value)
        self.mlen = float(self.get_parameter('marker_length_m').value)

        self.rvec = None
        self.tvec = None

        # Publisher for the marker pose
        self.marker_pub = self.create_publisher(Float64MultiArray, '/marker_pose', 10)

        # dictionary (compat)
        #dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_6X6_250)
        dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)  # This is the dict in launch file
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(dict_id)

        # detector params (compat)
        try:
            self.detector_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.detector_params = cv2.aruco.DetectorParameters_create()

        # intrinsics
        self.K = None; self.D = None
        self.bridge = CvBridge()
        self.create_subscription(CameraInfo, ct, self.on_info, 1)
        self.create_subscription(Image, it, self.on_img, 10)

        cv2.namedWindow('aruco', cv2.WINDOW_NORMAL)
        self.get_logger().info('aruco_view running - press q/Esc to quit')

    def publish_marker_pose(self):
        msg = Float64MultiArray()

        if self.tvec is not None:
            rmat, _ = cv2.Rodrigues(self.rvec)
            tvec = self.tvec

            rmat = rmat.flatten()

            for r in rmat:
                msg.data.append(r)
            for t in tvec:
                msg.data.append(t)

        self.marker_pub.publish(msg)

    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3,3)
        self.D = np.array(msg.d, dtype=np.float64).reshape(-1,1) if msg.d else np.zeros((5,1))

    def on_img(self, msg: Image):
        if self.K is None: return
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        corners, ids, _ = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.detector_params)

        if ids is not None and len(ids) > 0:
            ids = ids.flatten()
            idx = int(np.where(ids == self.want_id)[0][0]) if (self.want_id in ids and self.want_id >= 0) else 0
            mc = [corners[idx]]

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(mc, self.mlen, self.K, self.D)

            self.rvec = np.squeeze(rvec)
            self.tvec = np.squeeze(tvec)

            cv2.aruco.drawDetectedMarkers(img, mc, np.array([[ids[idx]]]))
            cv2.drawFrameAxes(img, self.K, self.D, rvec, tvec, self.mlen*0.5)

        else:
            self.rvec = None
            self.tvec = None

        self.publish_marker_pose()

        cv2.imshow('aruco', img)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(ArucoView())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()