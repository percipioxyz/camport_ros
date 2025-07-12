#!/usr/bin/env python
import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import threading
import sys
import os
import time
from datetime import datetime

class PointCloudViewer3D:
    def __init__(self):
        rospy.init_node('pointcloud_3d_viewer', anonymous=True)
        
        # 点云可视化参数
        self.point_size = rospy.get_param('~point_size', 1.0)
        self.background_color = [0.1, 0.1, 0.1]
        self.latest_cloud = None
        self.update_view = False
        
        # 点云保存参数
        self.save_path = rospy.get_param('~save_path', os.path.expanduser('./pointcloud_saves'))
        os.makedirs(self.save_path, exist_ok=True)
        rospy.loginfo(f"点云将保存至: {self.save_path}")
        
        # 设置PointCloud2订阅器
        pointcloud_topic = rospy.get_param('~pointcloud_topic', '/camera/PointCloud2')
        self.sub = rospy.Subscriber(pointcloud_topic, PointCloud2, self.pointcloud_callback)
        rospy.loginfo(f"订阅点云话题: {pointcloud_topic}")
        
        # 启动Open3D可视化线程
        threading.Thread(target=self.visualization_thread, daemon=True).start()
        
        # 打印使用说明
        rospy.loginfo("3D PointCloud Viewer Ready")
        rospy.loginfo("Press Ctrl+C to exit.")

    def pointcloud_callback(self, msg):
        """PointCloud2消息回调函数"""
        try:
            # 提取点云数据
            points = []
            colors = []
            
            # 从PointCloud2消息中提取点和颜色
            gen = point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
            
            for point in gen:
                # 提取坐标
                x, y, z = point[0], point[1], point[2]
                points.append([x, y, z])
                
                # 提取颜色（如果存在）
                #if len(point) > 3:
                #    rgb = point[3]
                    # 解析RGB（32位浮点数转RGB）
                #    r = 1.0
                #    g = 1.0
                #    b = 1.0
                colors.append([1.0, 1.0, 1.0])
            
            if not points:
                rospy.logwarn("收到空点云消息!")
                return
            
            # 创建Open3D点云
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.array(points))
            
            # 如果有颜色信息则添加
            if colors and len(colors) == len(points):
                pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
                
            
            # 下采样提高性能
            #if len(points) > 100000:
            #    pcd = pcd.voxel_down_sample(voxel_size=0.01)
            #    rospy.loginfo(f"点云下采样后点数: {len(pcd.points)}")
            
            self.latest_cloud = pcd
            self.update_view = True
            
        except Exception as e:
            rospy.logerr(f"处理点云错误: {str(e)}")
            exc_type, exc_obj, exc_tb = sys.exc_info()
            rospy.logerr(f"错误位置: 第 {exc_tb.tb_lineno} 行")

    def save_current_pointcloud(self, cloud):
        """保存当前点云到文件"""
        if cloud is None or len(cloud.points) == 0:
            rospy.logwarn("没有点云数据可保存!")
            return False
            
        # 生成带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"cloud_{timestamp}.ply"
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
            window_name='ROS PointCloud2 Viewer',
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
        cam = ctr.convert_to_pinhole_camera_parameters()
        extrinsics = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, -1],
            [0, 0, 0, 1]
        ])
        cam.extrinsic = extrinsics
        ctr.convert_from_pinhole_camera_parameters(cam)
        
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
                
                if first_frame:
                    self.save_current_pointcloud(self.latest_cloud)
                    first_frame = False
                elif prev_view_params:
                    # 恢复之前的视图状态
                    ctr.convert_from_pinhole_camera_parameters(prev_view_params)
            
            # 更新渲染
            vis.poll_events()
            vis.update_renderer()
            
            # 保存当前视图状态（用于下一次更新）
            current_view_params = ctr.convert_to_pinhole_camera_parameters()
            
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
        
        viewer = PointCloudViewer3D()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点已关闭")