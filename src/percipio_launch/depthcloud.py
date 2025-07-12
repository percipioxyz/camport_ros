#!/usr/bin/env python
import rospy
import numpy as np
import open3d as o3d
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import message_filters
import threading
import sys
import os
import time
from datetime import datetime  # 添加datetime用于生成时间戳文件名

def get_current_zoom(view_control):
    if hasattr(view_control, 'get_zoom'):
        return view_control.get_zoom()
    else:
        # 旧版本替代方法
        camera_params = view_control.convert_to_pinhole_camera_parameters()
        camera_pos = np.linalg.inv(camera_params.extrinsic)[:3, 3]
        return np.linalg.norm(camera_pos)
        
class DepthCloudViewer3D:
    def __init__(self):
        rospy.init_node('depthcloud_3d_viewer', anonymous=True)
        
        # 创建OpenCV转换器
        self.bridge = CvBridge()
        
        # 点云可视化参数
        self.point_size = rospy.get_param('~point_size', 1.0)
        self.background_color = [0.1, 0.1, 0.1]
        self.camera_params = None
        self.latest_cloud = None
        self.update_view = False
        
        # 点云保存参数
        self.save_path = rospy.get_param('~save_path', os.path.expanduser('./pointcloud_saves'))
        os.makedirs(self.save_path, exist_ok=True)
        rospy.loginfo(f"点云将保存至: {self.save_path}")
        
        # 设置ROS订阅器
        depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image')
        rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image')
        info_topic = rospy.get_param('~info_topic', '/camera/rgb/camera_info')
        
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        rgb_sub = message_filters.Subscriber(rgb_topic, Image)
        info_sub = message_filters.Subscriber(info_topic, CameraInfo)
        
        # 同步订阅
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [depth_sub, rgb_sub, info_sub], queue_size=10, slop=0.2
        )
        self.ts.registerCallback(self.image_callback)
        
        # 启动Open3D可视化线程
        threading.Thread(target=self.visualization_thread, daemon=True).start()
        
        # 打印使用说明
        rospy.loginfo(f"3D DepthCloud Viewer Ready. Topics: {depth_topic}, {rgb_topic}, {info_topic}")
        rospy.loginfo("按 'S' 保存当前点云 | 按 'Q' 退出 | 按 'P' 打印视角信息")
        rospy.loginfo("Press Ctrl+C to exit.")

    def image_callback(self, depth_msg, rgb_msg, info_msg):
        try:
            # 保存相机内参
            if self.camera_params is None:
                self.camera_params = {
                    'fx': info_msg.K[0],
                    'fy': info_msg.K[4],
                    'cx': info_msg.K[2],
                    'cy': info_msg.K[5],
                    'width': info_msg.width,
                    'height': info_msg.height
                }
                rospy.loginfo(f"相机参数: fx={self.camera_params['fx']}, fy={self.camera_params['fy']}, "
                              f"cx={self.camera_params['cx']}, cy={self.camera_params['cy']}")
            
            # 转换深度图
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            
            # 转换RGB图
            color_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            
            # 生成点云
            self.generate_pointcloud(depth_image, color_image)
            self.update_view = True
            
        except Exception as e:
            rospy.logerr(f"处理错误: {str(e)}")
            exc_type, exc_obj, exc_tb = sys.exc_info()
            rospy.logerr(f"错误位置: 第 {exc_tb.tb_lineno} 行")

    def generate_pointcloud(self, depth_image, color_image):
        """生成点云并优化性能"""
        fx, fy = self.camera_params['fx'], self.camera_params['fy']
        cx, cy = self.camera_params['cx'], self.camera_params['cy']
        height, width = depth_image.shape
        
        # 使用向量化操作替代循环
        u = np.arange(width)
        v = np.arange(height)
        u, v = np.meshgrid(u, v)
        
        # 转换为3D坐标
        z = depth_image.astype(np.float32) / 1000.0  # 转为米
        valid_mask = (z > 0) & (z < 5.0)  # 过滤无效点
        
        if not np.any(valid_mask):
            rospy.logwarn("未找到有效的深度点!")
            return
            
        x = (u[valid_mask] - cx) * z[valid_mask] / fx
        y = (v[valid_mask] - cy) * z[valid_mask] / fy
        z_valid = z[valid_mask]
        
        # 创建点云
        points = np.column_stack((x, y, z_valid))
        colors = color_image[valid_mask] / 255.0
        
        # 创建Open3D点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # 下采样提高性能
        #if len(points) > 50000:
        #    rospy.loginfo(f"点云下采样前点数: {len(points)}")
        #    pcd = pcd.voxel_down_sample(voxel_size=0.02)
        #    rospy.loginfo(f"点云下采样后点数: {len(pcd.points)}")
        
        self.latest_cloud = pcd

    def save_current_pointcloud(self, cloud):
        """保存当前点云到文件"""
        if cloud is None or len(cloud.points) == 0:
            rospy.logwarn("没有点云数据可保存!")
            return False
            
        # 生成带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"depthcloud_{timestamp}.ply"
        filepath = os.path.join(self.save_path, filename)
        
        try:
            # 保存点云
            o3d.io.write_point_cloud(filepath, cloud)
            rospy.loginfo(f"点云已保存至: {filepath}")
            rospy.loginfo(f"点数: {len(cloud.points)}")
            return True
        except Exception as e:
            rospy.logerr(f"保存点云失败: {str(e)}")
            return False

    def visualization_thread(self):
        """Open3D可视化线程"""
        # 延迟初始化解决OpenGL上下文问题
        rospy.sleep(1.0)
        
        # 创建可视化窗口
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name='ROS DepthCloud Viewer',
            width=1280,
            height=720,
            visible=True
        )
        
        # 初始空点云
        dummy_cloud = o3d.geometry.PointCloud()
        dummy_cloud.points = o3d.utility.Vector3dVector(np.array([[0,0,0]]))
        vis.add_geometry(dummy_cloud)
        
        # 设置渲染选项
        render_opt = vis.get_render_option()
        render_opt.background_color = np.array(self.background_color)
        render_opt.point_size = self.point_size
        render_opt.light_on = True
        
        # 设置默认视角
        ctr = vis.get_view_control()
        ctr.set_front([0, -1, 0.5])  # 相机朝向 (X:右, Y:下, Z:前)
        ctr.set_up([0, 0, 1])        # 上方向 (Z轴向上)
        ctr.set_zoom(0.4)            # 初始缩放
        
        current_view_params = None
        first_frame = True
        
        while not rospy.is_shutdown():
            # 更新点云显示
            if self.update_view and self.latest_cloud is not None:
                # 保存当前视图状态（如果已有）
                if current_view_params:
                    prev_view_params = ctr.convert_to_pinhole_camera_parameters()
                
                # 更新点云
                vis.clear_geometries()
                vis.add_geometry(self.latest_cloud)
                self.update_view = False
                
                # 智能设置初始视角
                if first_frame:
                    # 计算点云的边界框
                    bbox = self.latest_cloud.get_axis_aligned_bounding_box()
                    center = bbox.get_center()
                    extent = bbox.get_extent()
                    
                    # 设置智能初始视角
                    ctr.set_lookat(center)  # 看向点云中心
                    
                    # 根据点云大小自动设置距离
                    max_extent = max(extent[0], extent[1], extent[2])
                    distance = max_extent * 1.8  # 保持适当距离
                    
                    # 设置视角位置
                    eye_pos = center + np.array([distance, distance, distance])
                    ctr.set_front(eye_pos - center)
                    
                    # 设置向上方向
                    ctr.set_up([0, 0, 1]) 
                    
                    # 设置缩放
                    zoom_level = 0.5 * (1.0 / max_extent) if max_extent > 0 else 0.4
                    ctr.set_zoom(zoom_level)

                    self.key_s_callback(vis)
                    
                    first_frame = False
                    #rospy.loginfo(f"设置智能初始视角: 距离={distance:.2f}m, 缩放={zoom_level:.2f}")
                elif prev_view_params:
                    # 恢复之前的视图状态
                    ctr.convert_from_pinhole_camera_parameters(prev_view_params)
            
            # 更新渲染
            vis.poll_events()
            vis.update_renderer()
            
            # 保存当前视图状态（用于下一次更新）
            current_view_params = ctr.convert_to_pinhole_camera_parameters()
            
            # 显示当前视角信息
            zoom = get_current_zoom(ctr)
            if self.latest_cloud:
                point_count = len(self.latest_cloud.points) if self.latest_cloud else 0
                title = f"ROS深度点云 - 点数: {point_count} | 缩放: {zoom:.2f} | 按 S 保存"
                #vis.set_window_title(title)
            
            rospy.sleep(0.05)
        
        # 关闭窗口
        vis.destroy_window()
        rospy.loginfo("Open3D可视化已关闭")

    def key_s_callback(self, vis):
        """按键'S'的回调函数 - 保存点云"""
        if self.latest_cloud is not None:
            self.save_current_pointcloud(self.latest_cloud)
        else:
            rospy.logwarn("没有可用的点云数据!")
        return False

if __name__ == '__main__':
    try:
        # 解决Open3D在ROS环境中的初始化问题
        import platform
        if platform.system() == 'Linux':
            if 'DISPLAY' not in os.environ:
                rospy.logwarn("未设置DISPLAY环境变量，尝试设置为 ':0'")
                os.environ['DISPLAY'] = ':0'
        print(o3d.__version__) 
        viewer = DepthCloudViewer3D()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点已关闭")