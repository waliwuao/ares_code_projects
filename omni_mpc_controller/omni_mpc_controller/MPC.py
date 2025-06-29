import math
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
import numpy as np
from sklearn.cluster import DBSCAN
from tf_transformations import euler_from_quaternion
from scipy.optimize import linear_sum_assignment
from scipy.optimize import minimize
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from std_msgs.msg import ColorRGBA

class Transformation :
    def __init__(self,angle,turn_over,dimension = 2):
        """
        1.angle: (rad) means rolling form the asix x(original) to x'(target).
        Considering positive and negative direction
        2.turn_over: (True/False) True means the transformation is a turn over along y axis.
        3.dimension: (int) 2D or 3D transformation. Default is 2D.
        """
        self.angle = angle 
        self.turn_over = turn_over
        self.dimension = dimension
        
    def preparation(self):
        # the transformation matrix is a rotation matrix
        # considering the angle and the turn over
        if self.dimension == 2:
            T = np.matrix([[np.cos(self.angle), -np.sin(self.angle)],
                           [np.sin(self.angle),  np.cos(self.angle)]])
            if self.turn_over == True:
                Q = np.matrix([[1, 0],
                               [0,-1]])
                T = Q * T
            return T
        
    def transformation(self,point,dim):
        if dim ==  2:
            T = self.preparation()
            point = T * point
        return point

transformation = Transformation(-np.pi/2,True,2)
"""
坐标系
                                ----------------
                                |              |
                                |              |
                                |              |
                                |              |
                                |              |
                            15  |              |
                                |              |
                                |              |
                                |              |
                                |              |     ^
                                |              |     |x
                                ----------------  <---  
                                        8           y
"""

# 设置字体 (确保系统中有对应的字体，支持英文字符)
# Set font (ensure corresponding font is in the system, supports English ]characters)
# try:
#     # Prioritize common universal fonts that are likely to be present
#     plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans']
#     # Keep this for displaying minus signs correctly
#     # 保持这个设置以正确显示负号
#     plt.rcParams['axes.unicode_minus'] = False
# except Exception as e:
#     print(f"Warning: Could not set preferred font. Error: {e}")
#     print("Using default font.")
#     # Fallback to system default
#     # 回退到系统默认字体
#     plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']


class CarController(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('car_controller')
        # 创建订阅者 
        """节点名称根据具体2D雷达的topic名称来设置"""
        self.subscription_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)#10是队列大小
        self.subscription_position = self.create_subscription(
            PoseStamped,
            '/laser_position',
            self.position_callback,
            10
        )

        self.keyboard_control = self.create_subscription(
            Point,
            '/input_coordinates',
            self.goal_callback,
            10
        )

        self.controller_speed = self.create_subscription(
            Twist,
            '/cmd_vel_remote',
            self.speed_callback,
            10
        )
        
        self.obstacle_pub = self.create_publisher(
            MarkerArray,
            '/visualization/obstacles',
            10
        )

        self.scan_pub = self.create_publisher(
            Marker,
            '/visualization/filtered_scan',
            10
        )

        self.trajectory_pub = self.create_publisher(
            Marker,
            '/visualization/mpc_trajectory',
            10
        )

        self.goal_pub = self.create_publisher(
            Marker,
            '/visualization/goal',
            10
        )

        self.obstacle_trajectory_pub = self.create_publisher(
            MarkerArray,
            '/visualization/obstacle_trajectories',
            10
        )

        self.acceleration_pub = self.create_publisher(
            Marker,
            '/visualization/mpc_acceleration',
            10
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('dt', 0.00),
                ('max_speed', 0.0),
                ('max_accel', 0.0),
                ('weight_goal', 00.0),
                ('weight_velocity', 0.0),
                ('weight_accel', 0.0),
                ('weight_obstacle', 0.0),
                ('weight_boundary', 0.0),
                ('car_radius', 0.0),
                ('obstacle_visual_radius', 0.0),
                ('boundary_safe_distance', 0.0),
                ('avoidance_margin', 0.0),
                ('warning_zone_distance', 0.0),
                ('prediction_horizon', 0),
                ('world_size', [0.00,0.00]),
                ('dbscan_eps', 0.0),
                ('dbscan_min_samples', 0),
                ('max_history_length', 0.0),
                ('max_expected_obstacle_speed', 0.0)
            ]
        )
        
        self.vx = 0.0
        self.vy = 0.0
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.acceleration = self.get_parameter('max_accel').get_parameter_value().double_value # This parameter is not currently used by the MPC logic, which commands acceleration directly
        self.trajectory = []
        self.prediction_horizon = self.get_parameter('prediction_horizon').get_parameter_value().integer_value
        self.max_accel = self.get_parameter('max_accel').get_parameter_value().double_value
        self.weight_goal = self.get_parameter('weight_goal').get_parameter_value().double_value        # Velocity matching weight (optional, makes the car tend towards a certain speed related to manual direction)
        self.weight_velocity = self.get_parameter('weight_velocity').get_parameter_value().double_value # Weight for velocity tracking in manual mode
        self.weight_accel = self.get_parameter('weight_accel').get_parameter_value().double_value
        self.weight_obstacle = self.get_parameter('weight_obstacle').get_parameter_value().double_value
        self.weight_boundary = self.get_parameter('weight_boundary').get_parameter_value().double_value
        self.car_radius = self.get_parameter('car_radius').get_parameter_value().double_value
        self.obstacle_visual_radius = self.get_parameter('obstacle_visual_radius').get_parameter_value().double_value
        self.boundary_safe_distance = self.get_parameter('boundary_safe_distance').get_parameter_value().double_value
        self.avoidance_margin = self.get_parameter('avoidance_margin').get_parameter_value().double_value
        self.warning_zone_distance = self.get_parameter('warning_zone_distance').get_parameter_value().double_value
        self.world_size = self.get_parameter('world_size').get_parameter_value().double_array_value
        self.blend_safe_dist = self.car_radius + self.obstacle_visual_radius + self.avoidance_margin
        self.manual_override_dist = self.blend_safe_dist + 3.0 # Increased blend range slightly
        self.blend_alpha = 0.0
        self.laser = []
        self.obstacle_clusters = []
        self.obstacle_history = {}
        self.max_history_length = self.get_parameter('max_history_length').get_parameter_value().double_value
        self.dbscan_eps = self.get_parameter('dbscan_eps').get_parameter_value().double_value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').get_parameter_value().integer_value
        self.max_expected_obstacle_speed = self.get_parameter('max_expected_obstacle_speed').get_parameter_value().double_value
        self.x_initial = 0.0
        self.y_initial = 0.0
        self.x = 0.0
        self.y = 0.0 
        self.vx_initial = 0.0
        self.vy_initial = 0.0
        self.theta = 0.0
        self.speed = np.array([0.0,0.0])
        self.goal = None
        self.speed_1 = []
        self.angular_cmd = 0

        self.get_logger().info(f"""
        dt: {self.dt}
        max_speed: {self.max_speed}
        max_accel: {self.max_accel}
        weight_goal: {self.weight_goal}
        weight_velocity: {self.weight_velocity}
        weight_accel: {self.weight_accel}
        weight_obstacle: {self.weight_obstacle}
        weight_boundary: {self.weight_boundary}
        car_radius: {self.car_radius}
        obstacle_visual_radius: {self.obstacle_visual_radius}
        boundary_safe_distance: {self.boundary_safe_distance}
        avoidance_margin: {self.avoidance_margin}
        warning_zone_distance: {self.warning_zone_distance}
        world_size: {self.world_size}
        dbscan_eps: {self.dbscan_eps}
        dbscan_min_samples: {self.dbscan_min_samples}
        max_history_length: {self.max_history_length}
        max_expected_obstacle_speed: {self.max_expected_obstacle_speed}
        """)

        # 定时器，每dt秒执行一次，执行time_callback函数
        self.timer = self.create_timer(self.dt, self.time_callback)

        """发布决策信息,其中的消息类型需要根据需求进行修改"""
        self.publisher_ = self.create_publisher(
            Twist,
            '/mpc_decision',
            10
        )

    def speed_callback(self,msg):
        self.speed_1[0] = msg.linear.x
        self.speed_1[1] = msg.linear.y
        self.angular_cmd = msg.angular.x
        self.get_logger().info(f"{self.angular_cmd},{self.speed_1}")
        self.speed_1 = transformation.transformation(self.speed_1)

    def scan_callback(self, msg):
        """2D激光雷达数据回调函数"""
        # 获取激光雷达数据并且进行可视化，使用matplotlib
        # self.get_logger().info('Laser scan received')
        # 计算距离最近的障碍物距离

        self.laser = []
        
        if len(msg.ranges) != 0:
            #self.get_logger().info("--------------------------------Laser scan received--------------------------------")
            i = 0
            for distance in msg.ranges:
                if distance < 0.0 or distance == np.inf or distance == np.nan or (i > 1360 and i < 2450):
                    self.laser.append(1000000)
                    i += 1
                else:
                    self.laser.append(distance)
                    i += 1

        else:
            self.get_logger().info("--------------------------------No No No No--------------------------------: No laser data")

    def create_mpc_trajectory_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # 线宽
        
        # 设置颜色 (绿色)
        marker.color = ColorRGBA()
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # 设置持续时间
        duration = Duration()
        duration.sec = 0
        duration.nanosec = int(0.5 * 1e9)  # 0.5秒
        marker.lifetime = duration
        
        # 添加轨迹点
        if hasattr(self, 'mpc_trajectory') and self.mpc_trajectory:
            for point in self.mpc_trajectory:
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = 0.0
                marker.points.append(p)
        
        return marker

    def create_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 设置位置
        marker.pose.position.x = float(self.goal[0])
        marker.pose.position.y = float(self.goal[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 设置大小
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # 设置颜色 (红色)
        marker.color = ColorRGBA()
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # 设置持续时间
        duration = Duration()
        duration.sec = 0
        duration.nanosec = int(1.0 * 1e9)  # 1秒
        marker.lifetime = duration
        
        return marker        


    def create_acceleration_marker(self, ax, ay):
        """创建加速度向量可视化标记"""                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mpc_acceleration"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # 设置起点为小车当前位置
        start_point = Point()
        start_point.x = float(self.x)
        start_point.y = float(self.y)
        start_point.z = 0.0
        
        # 设置终点为加速度向量末端位置
        end_point = Point()
        end_point.x = float(self.x + ax * 0.5)  # 缩放因子0.5使箭头长度适中
        end_point.y = float(self.y + ay * 0.5)
        end_point.z = 0.0
        
        marker.points.append(start_point)
        marker.points.append(end_point)
        
        # 设置箭头样式
        marker.scale.x = 0.1  # 箭头轴直径
        marker.scale.y = 0.2  # 箭头头直径
        marker.scale.z = 0.0  # 不使用
        
        # 设置颜色 (蓝色)
        marker.color = ColorRGBA()
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # 设置持续时间
        duration = Duration()
        duration.sec = 0
        duration.nanosec = int(0.1 * 1e9)  # 0.1秒
        marker.lifetime = duration
        
        return marker


    def create_obstacle_trajectory_markers(self):
        marker_array = MarkerArray()
        
        for i, cluster in enumerate(self.obstacle_clusters):
            # 障碍物当前位置标记
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacle_trajectories"
            marker.id = i * 2  # 使用偶数ID表示障碍物当前位置
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cluster['center'][0]
            marker.pose.position.y = cluster['center'][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            duration = Duration()
            duration.sec = 0
            duration.nanosec = int(0.5 * 1e9)
            marker.lifetime = duration
            marker_array.markers.append(marker)
            
            # 障碍物预测轨迹标记
            if cluster['id'] in self.obstacle_history and len(self.obstacle_history[cluster['id']]) >= 2:
                traj_marker = Marker()
                traj_marker.header.frame_id = "map"
                traj_marker.header.stamp = self.get_clock().now().to_msg()
                traj_marker.ns = "obstacle_trajectories"
                traj_marker.id = i * 2 + 1  # 使用奇数ID表示轨迹
                traj_marker.type = Marker.LINE_STRIP
                traj_marker.action = Marker.ADD
                traj_marker.scale.x = 0.03
                traj_marker.color.a = 0.7
                traj_marker.color.r = 0.0
                traj_marker.color.g = 0.5
                traj_marker.color.b = 1.0
                traj_marker.lifetime = duration
                
                # 添加历史点作为轨迹
                for point in self.obstacle_history[cluster['id']]:
                    p = Point()
                    p.x = float(point[0])
                    p.y = float(point[1])
                    p.z = 0.0
                    traj_marker.points.append(p)
                
                marker_array.markers.append(traj_marker)
        
        return marker_array
    
    def create_obstacle_markers(self):
        marker_array = MarkerArray()
        
        for i, cluster in enumerate(self.obstacle_clusters):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cluster['center'][0]
            marker.pose.position.y = cluster['center'][1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            # 修改这里：正确创建Duration对象
            duration = Duration()
            duration.sec = 0
            duration.nanosec = int(0.5 * 1e9)  # 0.5秒转换为500000000纳秒
            marker.lifetime = duration
            
            marker_array.markers.append(marker)
        
        return marker_array

    def create_scan_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "filtered_scan"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        # 修改这里：正确创建Duration对象
        duration = Duration()
        duration.sec = 0
        duration.nanosec = int(0.1 * 1e9)  # 0.1秒转换为100000000纳秒
        marker.lifetime = duration
        
        for point in self.scan_points_for_plot:
            p = Point()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = 0.0
            marker.points.append(p)
        
        return marker
        
    def position_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        quaternion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        self.theta = euler_from_quaternion(quaternion)[2]
    
    def goal_callback(self, msg):
        self.goal[0] = msg.x
        self.goal[1] = msg.y

    def convert_polar_to_cartesian(self, angle_deg_relative, distance, car_x, car_y, car_theta_rad):
        """将相对于小车的极坐标点 (角度(度), 距离) 转换为世界坐标系的 (x, y)。"""
        # 首先检查距离是否大于10，如果是则返回None
        if distance > 10:
            return None
        
        # Converts a polar point (angle_deg, distance) relative to the car to world coordinates (x, y).
        # Use car's orientation to convert relative angle to world angle
        # car_theta_rad = math.radians(car_theta_deg)
        angle_rad_relative = math.radians(angle_deg_relative)

        angle_rad_world = car_theta_rad + angle_rad_relative
        
        #激光编号顺序为逆时针旋转
        world_x = car_x + distance * math.cos(angle_rad_world)
        world_y = car_y + distance * math.sin(angle_rad_world)
        return [world_x, world_y]


    def cluster_scan_points(self):
        """将原始雷达数据 (角度, 距离) 转换为世界坐标 (x, y) 点，并进行聚类。"""
        world_scan_points = []
        
        for i in range(len(self.laser)):
            point = self.convert_polar_to_cartesian(i*360/3240, self.laser[i], self.x, self.y, self.theta)
            if point is not None:  # 只添加非None的点
                world_scan_points.append(point) 
        
        if world_scan_points:  # 确保列表不为空
            self.scan_points_for_plot = np.array(world_scan_points)
        else:
            self.scan_points_for_plot = np.empty((0, 2))
            return []

        # Perform DBSCAN clustering on the world coordinates
        dbscan = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples)
        clusters = dbscan.fit_predict(self.scan_points_for_plot)
        
        clustered_points_map = {}
        for i, label in enumerate(clusters):
            if label != -1: # Ignore noise points (-1)
                if label not in clustered_points_map:
                    clustered_points_map[label] = []
                clustered_points_map[label].append(self.scan_points_for_plot[i])

        obstacle_centers = []
        car_pos = np.array([self.x, self.y])
        
        for label, points in clustered_points_map.items():
            center = np.mean(points, axis=0)
            # 计算障碍物中心点与小车的距离
            dist_to_car = np.linalg.norm(center - car_pos)
            # 只保留距离大于1米的障碍物
            if dist_to_car >= 0.5:
                obstacle_centers.append(center)
        
        #self.get_logger().info(f"障碍物中心点: {obstacle_centers}")    

        return obstacle_centers


    def track_obstacles(self, current_obstacle_centers, dt):
        """跟踪障碍物，并更新历史位置。使用匈牙利算法匹配当前检测到的中心与已知障碍物。"""
        # Tracks obstacles and updates historical positions. Uses the Hungarian algorithm to match current detected centers with known obstacles.
        # 获取已知障碍物的数量
        num_tracked = len(self.obstacle_clusters)
        # 获取当前检测到的障碍物的数量
        num_detected = len(current_obstacle_centers)

        # Build cost matrix for matching: Distance between tracked centers and detected centers
        #np.full 生成一个 num_tracked * num_detected 的矩阵，每次初始化的时候矩阵的值为无穷大，
        # 用于存储已知障碍物与当前检测到的障碍物之间的距离，
        cost_matrix = np.full((num_tracked, num_detected), np.inf)
        # 计算一个障碍物在 dt 时间内可能移动的最大距离
        # Threshold calculation: Max distance an object *could* move between frames + margin
        # Use actual frame dt for tracking threshold
        # 用最大可能的障碍物速度乘以 dt 得到一个障碍物在 dt 时间内可能移动的最大距离
        max_possible_movement = self.max_expected_obstacle_speed * dt
        # Add margin for robustness due to tracking imperfections, clustering noise, and potential small jitters
        # 添加一个缓冲区，用于考虑跟踪误差、聚类噪声以及潜在的小抖动
        match_threshold = max_possible_movement + self.dbscan_eps * 1.5 # Example margin based on DBSCAN epsilon


        for i in range(num_tracked):
            for j in range(num_detected):
                # linalg.norm 计算两个点之间的欧几里得距离
                dist = np.linalg.norm(np.array(self.obstacle_clusters[i]['center']) - np.array(current_obstacle_centers[j]))
                # Only consider potential matches within the threshold distance
                # 如果两个障碍物之间的距离小于匹配阈值，则认为这两个障碍物是同一个障碍物
                if dist < match_threshold:
                    #既然是同一个障碍物，那么代价矩阵的值就为两个障碍物之间的距离
                    cost_matrix[i, j] = dist # Use distance as cost

        updated_clusters = []
        matched_detected_indices = set() # Indices of detected centers that found a match
        matched_tracked_indices = set() # Indices of tracked clusters that found a match

        if num_tracked > 0 and num_detected > 0:#如果已知障碍物和当前检测到的障碍物都存在
            try:
                # Solve the assignment problem (Hungarian algorithm)
                # 使用匈牙利算法解决分配问题
                # linear_sum_assignment 返回一个行索引列表和列索引列表，
                row_ind, col_ind = linear_sum_assignment(cost_matrix)

                # Process matched pairs
                # 遍历已知障碍物和当前检测到的障碍物的匹配结果
                for i, j in zip(row_ind, col_ind):
                    # Check if the match is valid (cost < infinity, meaning it was within threshold)
                    # 如果两个障碍物之间的距离小于匹配阈值，则认为这两个障碍物是同一个障碍物
                    if cost_matrix[i, j] < np.inf:
                        # 获取已知障碍物，数据类型为字典
                        tracked_cluster = self.obstacle_clusters[i]
                        # 获取当前检测到的障碍物，数据类型为列表
                        detected_center = current_obstacle_centers[j]
                        # Update tracked cluster's center to the new detected position and reset age
                        # 更新已知障碍物的中心点为当前检测到的障碍物的中心点，并重置年龄
                        tracked_cluster['center'] = detected_center
                        tracked_cluster['age'] = 0

                        obstacle_id = tracked_cluster['id']
                        # Update history for velocity estimation
                        if obstacle_id not in self.obstacle_history:
                            # 如果已知障碍物不在历史记录中，则创建一个空列表
                            self.obstacle_history[obstacle_id] = []
                        # 将当前检测到的障碍物的中心点添加到已知障碍物的历史记录中
                        self.obstacle_history[obstacle_id].append(detected_center)
                        #进行实时数据更新，获取新数据的同时，删除旧数据
                        # Keep history length limited
                        if len(self.obstacle_history[obstacle_id]) > self.max_history_length:
                            self.obstacle_history[obstacle_id].pop(0) # Remove the oldest entry
                        # 将已知障碍物添加到更新列表中
                        updated_clusters.append(tracked_cluster)
                        # 将已知障碍物的索引添加到已知障碍物的索引列表中
                        matched_tracked_indices.add(i)
                        # 将当前检测到的障碍物的索引添加到当前检测到的障碍物的索引列表中
                        matched_detected_indices.add(j)

            except ValueError as e:
                # This can occur if the cost matrix contains no finite values (e.g., all distances > match_threshold)
                # or if the matrix dimensions are invalid.
                # print(f"Warning: linear_sum_assignment failed: {e}. Processing unmatched clusters/detections directly.")
                pass # Handle unmatched processing below

        # 处理新障碍物（检测到的但未匹配到任何已知障碍物的障碍物）
        # Handle new obstacles (detected but not matched to any tracked cluster)
        for j in range(num_detected):
            if j not in matched_detected_indices:
                new_center = current_obstacle_centers[j]
                # Assign a new unique ID
                existing_ids = list(self.obstacle_history.keys()) # Look through all history keys, even for aged-out ones, to avoid ID reuse
                new_id = max(existing_ids + [-1]) + 1 if existing_ids else 0
                new_cluster = {'id': new_id, 'center': new_center, 'age': 0}
                updated_clusters.append(new_cluster)
                # Start history for the new obstacle
                self.obstacle_history[new_id] = [new_center]


        # Handle lost obstacles (tracked but not matched to any detected center)
        # Age the tracked clusters that were not matched
        for i in range(num_tracked):
            if i not in matched_tracked_indices:
                lost_cluster = self.obstacle_clusters[i]
                lost_cluster.setdefault('age', 0) # Ensure 'age' key exists
                lost_cluster['age'] += 1
                # Keep the cluster for a few frames even if not detected, using its last known position
                if lost_cluster['age'] < 5: # Age threshold (e.g., 5 frames)
                    # Append the aged cluster with its last known position
                    updated_clusters.append(lost_cluster)
                else:
                    # If too old, remove from active tracking and history
                    if lost_cluster['id'] in self.obstacle_history:
                        del self.obstacle_history[lost_cluster['id']] # Remove history for this ID

        # Update the list of currently tracked obstacles
        self.obstacle_clusters = updated_clusters

    def calculate_blend_ratio(self, dist_to_nearest_obstacle):
        """Calculates blend ratio (alpha) between avoidance (1) and manual (0)."""
        # If no obstacles are tracked or the nearest is far away, pure manual/goal control (alpha=0)
        if dist_to_nearest_obstacle is None or dist_to_nearest_obstacle >= self.manual_override_dist:
            return 0.0
        # If an obstacle is very close, pure avoidance control (alpha=1)
        if dist_to_nearest_obstacle <= self.blend_safe_dist:
            return 1.0

        # In the transition zone, blend linearly based on the distance
        range_dist = self.manual_override_dist - self.blend_safe_dist
        if range_dist <= 1e-9: # Avoid division by zero if the range is tiny
            return 1.0 if dist_to_nearest_obstacle <= self.blend_safe_dist else 0.0

        # Clamp distance within the blending range for calculation
        clamped_dist = np.clip(dist_to_nearest_obstacle, self.blend_safe_dist, self.manual_override_dist)

        # Calculate alpha: 1 at blend_safe_dist, 0 at manual_override_dist
        # This formula maps [blend_safe_dist, manual_override_dist] to [1, 0]
        alpha = (self.manual_override_dist - clamped_dist) / range_dist

        # Ensure alpha is strictly between 0 and 1 (although clip should handle this)
        return np.clip(alpha, 0.0, 1.0)

    def motion_model(self, state, control, dt):
        """车辆运动模型：state = [x, y, vx, vy]. control = [ax_control, ay_control]. 包含基于当前速度的阻力项。"""
        x, y, vx, vy = state
        ax_control, ay_control = control

        # 计算到目标点的距离和方向
        if self.goal is not None:
            goal_pos = np.array(self.goal)
            current_pos = np.array([x, y])
            dist_to_goal = np.linalg.norm(goal_pos - current_pos)
            direction_to_goal = (goal_pos - current_pos) / (dist_to_goal + 1e-6)  # 避免除以零
            
            # 根据距离动态调整加速度
            # 当距离大于5米时，使用最大加速度
            # 当距离小于2米时，开始减速
            if dist_to_goal > 5.0:
                accel_scale = 1.0  # 最大加速度
            elif dist_to_goal < 2.0:
                # 在2米内开始减速，距离越近减速越快
                accel_scale = dist_to_goal / 2.0
            else:
                # 在2-5米之间平滑过渡
                accel_scale = 1.0
            
            # 应用加速度缩放
            ax_control *= accel_scale
            ay_control *= accel_scale

            # 确保加速度方向指向目标点
            current_speed = math.hypot(vx, vy)
            if current_speed < self.max_speed:
                # 如果速度未达到最大值，增加加速度
                ax_control += direction_to_goal[0] * self.max_accel * 0.5
                ay_control += direction_to_goal[1] * self.max_accel * 0.5


        # 计算净加速度
        net_ax = ax_control 
        net_ay = ay_control 

        # 更新速度
        next_vx = vx + net_ax * dt
        next_vy = vy + net_ay * dt

        # 应用速度限制
        current_speed_sq = next_vx**2 + next_vy**2
        max_speed_sq = self.max_speed**2
        if current_speed_sq > max_speed_sq:
            if current_speed_sq > 1e-12:
                scale = self.max_speed / math.sqrt(current_speed_sq)
                next_vx *= scale
                next_vy *= scale
            else:
                next_vx = 0.0
                next_vy = 0.0

        # 更新位置
        next_x = x + next_vx * dt
        next_y = y + next_vy * dt

        return np.array([next_x, next_y, next_vx, next_vy])


    def mpc_cost(self, u_flat, x0, goal, predicted_obstacle_paths):
        """MPC 优化目标函数。u_flat是展平的控制输入 [ax0, ay0, ax1, ay1, ...]."""
        # MPC objective function. u_flat is the flattened control input [ax0, ay0, ax1, ay1, ...].
        # u_flat: Control sequence [ax0, ay0, ax1, ay1, ...] from the optimizer (represents *control* acceleration, not net)
        # x0: initial state [x, y, vx, vy]
        # goal: target position [gx, gy] (numpy array or None)
        # predicted_obstacle_paths: list of paths, where each path is a list of predicted positions for each MPC step:
        #   [[pos_obs0_t0, pos_obs0_t1, ..., pos_obs0_tN], [pos_obs1_t0, pos_obs1_t1, ..., pos_obs1_tN], ...]
        # blend_alpha: Current blend ratio (1=avoidance, 0=manual/goal) - Accessed via self.blend_alpha

        cost = 0.0
        state = np.array(x0) # Current state at the start of the horizon

        # Store MPC predicted path for visualization
        # The trajectory starts from the current car position (x0[:2])
        self.mpc_trajectory = [state[:2].tolist()]

        alpha = self.blend_alpha # Get the current blend ratio

        # Scale weights based on the blending ratio
        # Obstacle cost is prioritized when alpha is high
        obstacle_cost_weight_scaled = alpha * self.weight_obstacle
        # Goal/Manual cost is prioritized when alpha is low
        # Use goal weight if a goal is set, otherwise use velocity weight for manual control
        goal_or_manual_weight_scaled = (1.0 - alpha) * (self.weight_goal if self.goal is not None else self.weight_velocity)


        # Iterate through each step in the MPC prediction horizon
        for i in range(self.prediction_horizon):
            # Extract *control* acceleration for the current step (i) from the optimizer's flattened input
            # Ensure indices are within the bounds of u_flat
            if (i * 2 + 1) < len(u_flat):
                control_i = np.array([u_flat[i * 2], u_flat[i * 2 + 1]])
            else:
                # Fallback to zero control if u_flat is shorter than expected (shouldn't happen with correct bounds/u0 size)
                control_i = np.array([0.0, 0.0])
                # print(f"Warning: u_flat index out of bounds at step {i}. Using zero control.")


            # Predict the next state of the car using the motion model and the control input for this step
            # The motion model includes the effect of resistance.
            # The state `state` is updated iteratively for each step.
            state = self.motion_model(state, control_i, self.dt)
            x, y, vx, vy = state # Unpack the state for easier access

            # Add the predicted position (x, y) at this step to the trajectory for visualization
            self.mpc_trajectory.append(state[:2].tolist())

            # --- Calculate Cost for this step ---

            # 1. Goal/Manual Control Cost (Weighted by 1-alpha)
            goal_or_manual_cost_step = 0.0
            if self.goal is not None:
                # If a goal is set, penalize distance to the goal (which is static)
                dist_to_goal = np.linalg.norm(state[:2] - self.goal)
                goal_or_manual_cost_step = dist_to_goal**2 # Quadratic penalty for distance
            else:
                # If no goal is set, penalize deviation from the desired manual velocity
                desired_velocity = self.speed * self.max_speed # Desired velocity vector
                velocity_error_sq = np.sum((state[2:] - desired_velocity)**2) # Squared error in velocity vector components
                goal_or_manual_cost_step = velocity_error_sq # Quadratic penalty for velocity error

            # Add the scaled goal/manual cost for this step to the total cost
            cost += goal_or_manual_weight_scaled * goal_or_manual_cost_step


            # 2. Control Effort Cost (Standard weight)
            # Penalize the magnitude of the *control* acceleration commanded by the optimizer
            cost += self.weight_accel * np.sum(control_i**2) # Quadratic penalty on control effort


            # 3. Obstacle Avoidance Cost (Weighted by alpha)
            obstacle_cost_step = 0.0
            # Only calculate obstacle cost if the scaled weight is significant and there are obstacle paths
            if obstacle_cost_weight_scaled > 1e-9 and predicted_obstacle_paths:
                # Define safe and warning distances for obstacles
                safe_distance = self.car_radius + self.obstacle_visual_radius + self.avoidance_margin
                warning_zone_dist_end = safe_distance + self.warning_zone_distance # Warning ends at this distance


                # Iterate through each obstacle's predicted path
                # predicted_obstacle_paths[j][i] is the predicted position of obstacle j at time t + i*dt
                for obs_path in predicted_obstacle_paths:
                    # Ensure the obstacle path has a predicted position for this specific step 'i'
                    if len(obs_path) > i:
                        obs_pos_at_i = np.array(obs_path[i]) # Get obstacle position at time t + i*dt

                        # Calculate the distance between the car's predicted position and the obstacle's predicted position at time t + i*dt
                        dist_to_obstacle = np.linalg.norm(state[:2] - obs_pos_at_i)

                        if dist_to_obstacle < safe_distance:
                            # High quadratic penalty for being inside or penetrating the safe distance (collision)
                            penetration = safe_distance - dist_to_obstacle
                            obstacle_cost_step += penetration**2 * 100 # Use a higher multiplier for collision

                        elif dist_to_obstacle < warning_zone_dist_end:
                            # Exponential penalty for being within the warning zone
                            # Penalty is high near safe_distance and approaches 0 near warning_zone_dist_end
                            # Penalty should be 0 when dist >= warning_zone_dist_end
                            # Let's use a sigmoid or related function, or a carefully constructed exponential
                            # A simple inverse distance penalty can also work: 1 / (dist - safe_distance) - 1 / (warning_zone_dist_end - safe_distance) when in range
                            # Or, exp((safe_distance - dist)/k)
                            # Let's refine the exponential penalty: exp(-(dist - safe_distance)/smoothing)
                            # At dist = safe_distance, exp(0) = 1
                            # At dist = warning_zone_dist_end = safe_distance + warning_zone_distance, exp(-warning_zone_distance/smoothing)
                            # We want penalty to be significant near safe_distance and negligible near warning_zone_dist_end.
                            # Use inverse exponential: penalty = exp((safe_distance - dist)/smoothing)
                            smoothing_factor = self.warning_zone_distance / 3.0 # Adjust smoothing based on warning zone width
                            obstacle_cost_step += math.exp((safe_distance - dist_to_obstacle) / smoothing_factor)
                            # An offset might be needed if we want it strictly 0 at warning_zone_dist_end, but a small non-zero value is often acceptable.


            # Add the scaled obstacle cost for this step to the total cost
            cost += obstacle_cost_weight_scaled * obstacle_cost_step


            # 4. Boundary Avoidance Cost (Standard weight)
            # Penalize getting too close to the world boundaries
            boundary_cost_step = 0.0
            # Calculate distance to each boundary
            dist_left = state[0] - (-self.world_size[0])
            dist_right = self.world_size[0] - state[0]
            dist_bottom = state[1] - (-self.world_size[1])
            dist_top = self.world_size[1] - state[1]

            boundary_safe_distance = self.boundary_safe_distance

            # Quadratic penalty if distance to a boundary is less than the safe distance
            if dist_left < boundary_safe_distance:
                boundary_cost_step += (boundary_safe_distance - dist_left)**2
            if dist_right < boundary_safe_distance:
                boundary_cost_step += (boundary_safe_distance - dist_right)**2
            if dist_bottom < boundary_safe_distance:
                boundary_cost_step += (boundary_safe_distance - dist_bottom)**2
            if dist_top < boundary_safe_distance:
                boundary_cost_step += (boundary_safe_distance - dist_top)**2

            # Add the boundary cost for this step to the total cost
            cost += self.weight_boundary * boundary_cost_step

        # The self.mpc_trajectory list is populated during the loop iterations

        return cost


    def predict_obstacle_positions(self):
        """根据历史位置预测障碍物在MPC预测步数内的路径。返回 list of [pos_t0, pos_t1, ..., pos_tN] for each obstacle."""
        # Predicts obstacle paths over the MPC prediction horizon based on historical positions. Returns list of paths.
        # Each path is a list of positions: [pos_at_t, pos_at_t+dt, pos_at_t+2*dt, ..., pos_at_t+N*dt]
        predicted_obstacle_paths = []

        for cluster in self.obstacle_clusters:
            obstacle_id = cluster['id']
            current_pos = np.array(cluster['center'])
            # Start the predicted path with the obstacle's current tracked position
            predicted_path = [current_pos.tolist()] # Position at time t (current time)

            # Default prediction is stationary (zero velocity)
            velocity_estimate = np.array([0.0, 0.0])

            # Estimate velocity if enough history points are available (at least 2 points)
            if obstacle_id in self.obstacle_history and len(self.obstacle_history[obstacle_id]) >= 2:
                history = self.obstacle_history[obstacle_id]
                # Use the two most recent history points to calculate a velocity vector
                pos_latest = np.array(history[-1])
                pos_previous = np.array(history[-2])

                # Time difference between the last two history points.
                # Using MPC dt here assumes tracking updates happen roughly aligned with MPC steps,
                # or that MPC dt is used as the time unit for velocity scaling.
                history_time_diff_approx = self.dt # Assume history points are approx MPC dt apart

                if history_time_diff_approx > 1e-9: # Avoid division by near zero
                    velocity_estimate = (pos_latest - pos_previous) / history_time_diff_approx

                # Cap the estimated speed to a maximum value to prevent unrealistic predictions from noise or tracking errors
                current_est_speed = np.linalg.norm(velocity_estimate)
                if current_est_speed > self.max_expected_obstacle_speed:
                    if current_est_speed > 1e-9: # Avoid division by zero
                        velocity_estimate = (velocity_estimate / current_est_speed) * self.max_expected_obstacle_speed
                    else:
                        velocity_estimate = np.array([0.0, 0.0]) # Speed is zero

            # Predict positions for each future time step in the MPC horizon (from step 1 to N)
            current_predicted_pos = current_pos.copy() # Prediction starts from current position
            for i in range(self.prediction_horizon):
                # Predict the position at the next time step (t + (i+1)*dt) assuming constant velocity
                current_predicted_pos = current_predicted_pos + velocity_estimate * self.dt
                predicted_path.append(current_predicted_pos.tolist()) # Add position at t + (i+1)*dt

            predicted_obstacle_paths.append(predicted_path)

        return predicted_obstacle_paths


    def solve_mpc(self, x0, goal, predicted_obstacle_paths):
        """求解 MPC 控制序列。"""
        # Solves for the MPC control sequence.
        # x0: Current state [x, y, vx, vy]
        # goal: Target position [gx, gy] or None
        # predicted_obstacle_paths: List of predicted paths for obstacles over the MPC horizon

        effective_goal = np.array(self.goal) if goal is not None else None
        # Initial guess for control inputs (zero acceleration for all steps)
        u0 = np.zeros(self.prediction_horizon * 2) # 2 control inputs (ax, ay) per step
        # Bounds for control inputs (limit acceleration magnitude for each axis, for each step)
        bounds = [(-self.max_accel, self.max_accel), (-self.max_accel, self.max_accel)] * self.prediction_horizon


        # Define the objective function that the optimizer will minimize
        # It takes the flattened control sequence u_flat and uses the other parameters (x0, goal, predicted_obstacle_paths)
        # which are available from the CarController object instance.
        objective = lambda u_flat: self.mpc_cost(u_flat, x0, effective_goal, predicted_obstacle_paths)

        # Use scipy.optimize.minimize with the SLSQP method
        # SLSQP supports bounds and is suitable for non-linear problems with constraints (though we only have bounds here)
        result = minimize(objective, u0,
                          method='SLSQP',
                          bounds=bounds,
                          options={'maxiter': 200, 'ftol': 1e-2, 'disp': False}) # *** Using Increased maxiter ***

        # The mpc_cost function (called by minimize) internally calculates and stores
        # the predicted trajectory (self.mpc_trajectory) for the *final* set of control inputs found by the optimizer.

        if result.success:
            # If optimization succeeded, return the optimal control sequence
            self.get_logger().info("succeeded")
            return result.x
        else:
            # If optimization failed, print a warning (optional) and return zero control inputs.
            # Returning zeros causes the vehicle to slow down due to resistance, effectively stopping it.
            # print(f"MPC Optimization failed: {result.message}") # Uncomment for debugging
            # Clear the MPC trajectory visualization as the planned path is not valid
            self.get_logger().info("fail")
        
            self.mpc_trajectory = []
            return np.zeros(self.prediction_horizon * 2) # Return zero control for all steps
        

    def time_callback(self):
        
        self.obstacle_centers = self.cluster_scan_points()
        self.track_obstacles(self.obstacle_centers, self.dt)
        self.vx = (self.x - self.x_initial) / self.dt
        self.vy = (self.y - self.y_initial) / self.dt
        self.x_initial = self.x
        self.y_initial = self.y
        # Calculate the distance from the car to the nearest currently *tracked* obstacle
        car_pos_np = np.array([self.x, self.y])
        d_min = float('inf') # Initialize min distance to infinity
        # If there are any currently tracked obstacle clusters
        if self.obstacle_clusters:
            # Calculate the distance from the car's position to the center of each tracked obstacle
            distances = [np.linalg.norm(car_pos_np - np.array(c['center'])) for c in self.obstacle_clusters]
            if distances: # Check if the list of distances is not empty
                d_min = min(distances) 
        self.blend_alpha = self.calculate_blend_ratio(d_min)
        predicted_obstacle_paths = self.predict_obstacle_positions() 
        current_state = np.array([self.x, self.y, self.vx, self.vy])      
        u_flat = self.solve_mpc(current_state, self.goal, predicted_obstacle_paths)

        #发送数据到决策节点，其中消息类型需要根据需求进行修改
        msg = Twist()
        self.get_logger().info(f"{u_flat[0],u_flat[1]}")
        commanded_vx = u_flat[0] * self.dt + self.vx_initial
        commanded_vy = u_flat[1] * self.dt + self.vy_initial
        self.vx_initial = commanded_vx
        self.vy_initial = commanded_vy 
        msg.linear.x = u_flat[0]
        msg.linear.y = u_flat[1]
        msg.angular.z = 0.0
        self.publisher_.publish(msg)


                # 发布障碍物可视化
        obstacle_markers = self.create_obstacle_markers()
        self.obstacle_pub.publish(obstacle_markers)

        # 发布过滤后的雷达数据可视化
        if hasattr(self, 'scan_points_for_plot'):
            scan_marker = self.create_scan_marker()
            self.scan_pub.publish(scan_marker)

        if hasattr(self, 'mpc_trajectory'):
            trajectory_marker = self.create_mpc_trajectory_marker()
            self.trajectory_pub.publish(trajectory_marker)

        if self.goal is not None:
            goal_marker = self.create_goal_marker()
            self.goal_pub.publish(goal_marker)

        obstacle_traj_markers = self.create_obstacle_trajectory_markers()
        self.obstacle_trajectory_pub.publish(obstacle_traj_markers)        

        acceleration_marker = self.create_acceleration_marker(u_flat[0], u_flat[1])
        self.acceleration_pub.publish(acceleration_marker)    

def main(args=None):
    rclpy.init(args=args)
    controller = CarController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()






