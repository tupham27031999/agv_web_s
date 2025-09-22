import open3d as o3d
import numpy as np
import cv2
import time
import math
import os



# --- PHẦN 1: CẤU HÌNH VÀ TẢI DỮ LIỆU ---

class Config:
    # BASE_PATH = r"data_input_output\scan_data_3\Scan_data_{}.npy"  
    BASE_PATH = r"data_input_output\scan_data_1\Scan_data_{}.npy"  
    OUTPUT_PCD = "global_map_offline.pcd"
    OUTPUT_OCCUPANCY_MAP = "realtime_occupancy_map.png" # ảnh cuối
    START_FILE = 1
    END_FILE = 1800
    RESOLUTION_MM_PER_PIXEL = 30 # ty le khoang cach ( độ phân giải)
    MAP_WIDTH_MM = 30000  # khoang cach toi da theo chiều ngang từ vị trí agv nhân 2
    MAP_HEIGHT_MM = 20000  # khoang cach toi da theo chiều dọc từ vị trí agv nhân 2
    ICP_VOXEL_SIZE = 30.0
    ICP_THRESHOLD = 200.0
    MAX_RMSE1_THRESHOLD = 80.0 # sai số lớn nhất
    MAX_RMSE2_THRESHOLD = 50.0 # sai số lớn nhất
    OUTLIER_NB_NEIGHBORS = 30
    OUTLIER_STD_RATIO = 2.0
    DUPLICATE_VOXEL_SIZE = 30.0
    DYNAMIC_DISTANCE_THRESHOLD = 250.0
    ROBOT_AXIS_LENGTH_MM = 300 # độ dài mũi tên
    DELAY_MS = 50 # delay waitkey
    MAP_WIDTH_PIXELS = int(MAP_WIDTH_MM / RESOLUTION_MM_PER_PIXEL) # kich thước ảnh theo chiều ngang
    MAP_HEIGHT_PIXELS = int(MAP_HEIGHT_MM / RESOLUTION_MM_PER_PIXEL) # kich thước ảnh theo chiều dọc
    # LOOP_CLOSURE_DISTANCE_THRESHOLD = 500.0  # Ngưỡng phát hiện loop closure (mm)

def load_and_prepare_scan(filepath):
    if not os.path.exists(filepath):
        print(f"Không tìm thấy tệp '{filepath}'")
        return None
    
    try:
        scan_data = np.load(filepath)
        if scan_data.ndim != 2 or scan_data.shape[1] not in [2, 3]:
            print(f"Định dạng không hợp lệ trong '{filepath}'. Shape: {scan_data.shape}")
            return None
        
        scan_data = np.asarray(scan_data, dtype=np.float64)
        
        if scan_data.shape[1] == 3:
            points_3d = polar_to_cartesian_3d(scan_data)
        else:
            print(f"Phát hiện định dạng cartesian 2D trong '{filepath}'. Đang thêm trục Z...")
            z_column = np.zeros((scan_data.shape[0], 1))
            points_3d = np.hstack((scan_data, z_column))
        
        return points_3d
    except Exception as e:
        print(f"Lỗi khi tải tệp '{filepath}': {str(e)}")
        return None

# --- PHẦN 2: CÁC HÀM SLAM ---

def polar_to_cartesian_3d(scan_data):
    if scan_data is None or len(scan_data) == 0:
        return np.array([])
    points_cartesian = []
    for point in scan_data:
        quality, angle, distance = point
        #is_in_front_arc = (angle <= 135) or (angle >= 225)
        if distance > 1000 and distance < 7000 and quality > 10 :#and is_in_front_arc:
            angle_rad = math.radians(angle)
            x = distance * math.cos(angle_rad)
            y = -distance * math.sin(angle_rad)
            points_cartesian.append([x, y, 0.0])
    return np.array(points_cartesian)

def lidar_to_point_cloud(points):
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    return point_cloud

def downsample_point_cloud(point_cloud, voxel_size):
    return point_cloud.voxel_down_sample(voxel_size)

def filter_outliers(point_cloud, nb_neighbors=10, std_ratio=2.5):
    pcd, _ = point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return pcd

def remove_duplicate_points(points, voxel_size=20.0):
    pcd = lidar_to_point_cloud(points)
    pcd_down = downsample_point_cloud(pcd, voxel_size)
    return np.asarray(pcd_down.points)

def remove_dynamic_points_bidirectional(current_points, prev_points, distance_threshold=250.0):
    if prev_points is None or len(prev_points) == 0:
        return current_points

    pcd_current = lidar_to_point_cloud(current_points)
    pcd_prev = lidar_to_point_cloud(prev_points)

    dist_current_to_prev = np.asarray(pcd_current.compute_point_cloud_distance(pcd_prev))
    dist_prev_to_current = np.asarray(pcd_prev.compute_point_cloud_distance(pcd_current))

    static_indices_current = np.where(dist_current_to_prev < distance_threshold)[0]
    static_indices_prev = np.where(dist_prev_to_current < distance_threshold)[0]

    static_current = pcd_current.select_by_index(static_indices_current)
    static_prev = pcd_prev.select_by_index(static_indices_prev)

    combined_static = static_current + static_prev
    return np.asarray(combined_static.points)

def transform_points(points, rotation_matrix, translation_vector):
    points = np.asarray(points)
    return np.dot(points, rotation_matrix.T) + translation_vector

def gicp(points1, points2, threshold=10,max_iteration=300, voxel_size=0.1, trans_init=np.eye(4)):
    ''' 
        voxel_size_old = 0.1 | đang test là 2: tạo 1 khung vuông 2x2 và gộp các điểm trong khung vuông
        max_iteration: số lần lặp tối đa
        max_nn + radius: số điểm tối đa max_nn nằm trong bán kính radius được xét (radius = 0.1(old) or 1)

        radius=1, max_nn=30, max_iteration=1000, voxel_size=2  --> tb 2.74
        radius=0.1, max_nn=30, max_iteration=1000, voxel_size=0.1 --> tb 2.79

        Tham số threshold chính là khoảng cách xa nhất mà một cặp điểm có thể được coi là một "cặp tương ứng hợp lệ" (inlier).
        có thể điều chỉnh threshold = 5
        
    '''
    source_pcd = lidar_to_point_cloud(points1)
    target_pcd = lidar_to_point_cloud(points2)

    source_pcd = downsample_point_cloud(source_pcd, voxel_size)
    target_pcd = downsample_point_cloud(target_pcd, voxel_size)


    # Tính toán các vector pháp tuyến cho PointCloud
    source_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Tính toán các ma trận hiệp phương sai cho PointCloud
    source_pcd.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target_pcd.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # Thiết lập các tham số cho GICP
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration)

    # Sử dụng GICP để căn chỉnh các điểm quét
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria)
    return reg_p2p.inlier_rmse, reg_p2p.transformation
def bresenham_line(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x1, y1))
    return points

def update_occupancy_map(occupancy_map, points_global, robot_pos, map_center_px, resolution,
                         p_occ_inc=0.2, p_free_dec=0.95): # tăng dần độ đậm (nếu điểm đen bị quét trắng nhiều lần sẽ nhạt dần)
    if len(points_global) == 0:
        return

    height, width = occupancy_map.shape[:2]

    if occupancy_map.ndim == 3 and occupancy_map.shape[2] == 3:
        if not hasattr(update_occupancy_map, "occupancy_probs"):
            update_occupancy_map.occupancy_probs = np.full((height, width), 0.5, dtype=np.float32)  # bắt đầu ở 0.5

        occ = update_occupancy_map.occupancy_probs
    else:
        occ = occupancy_map

    robot_x_px = int(map_center_px[0] + robot_pos[0] / resolution)
    robot_y_px = int(map_center_px[1] - robot_pos[1] / resolution)

    for pt in points_global:
        px = int(map_center_px[0] + pt[0] / resolution)
        py = int(map_center_px[1] - pt[1] / resolution)

        if not (0 <= px < width and 0 <= py < height):
            continue

        line = bresenham_line(robot_x_px, robot_y_px, px, py)

        for (x, y) in line[:-1]:
            if 0 <= x < width and 0 <= y < height:
                occ[y, x] = max(0.0, occ[y, x] * p_free_dec)

        x, y = line[-1]
        if 0 <= x < width and 0 <= y < height:
            occ[y, x] = min(1.0, occ[y, x] + p_occ_inc)

    if occupancy_map.ndim == 3 and occupancy_map.shape[2] == 3:
        occ_uint8 = ((1-occ) * 255).astype(np.uint8)
        occupancy_map[:, :, 0] = occ_uint8 
        occupancy_map[:, :, 1] = occ_uint8 
        occupancy_map[:, :, 2] = occ_uint8 
   
def scan_on_map(occupancy_map, points, map_center_px, resolution, color=(0, 255, 0)):
    for point in points:
        point_x_px = int(map_center_px[0] + point[0] / resolution)
        point_y_px = int(map_center_px[1] - point[1] / resolution)
        if 0 <= point_x_px < occupancy_map.shape[1] and 0 <= point_y_px < occupancy_map.shape[0]:
            cv2.circle(occupancy_map, (point_x_px, point_y_px), 1, color, -1)

def draw_robot_pose(occupancy_map, pose, map_center_px, resolution, axis_length=300):
    position = pose[:3, 3]
    rotation = pose[:3, :3]

    robot_x_px = int(map_center_px[0] + position[0] / resolution)
    robot_y_px = int(map_center_px[1] - position[1] / resolution) 
    robot_center_px = (robot_x_px, robot_y_px)

    x_axis_vec = np.array([axis_length, 0, 0])
    x_axis_end_vec = rotation @ x_axis_vec
    x_end_px = (int(robot_x_px + x_axis_end_vec[0] / resolution), int(robot_y_px - x_axis_end_vec[1] / resolution))    
    cv2.line(occupancy_map, robot_center_px, x_end_px, (0, 0, 255), 1)
    cv2.circle(occupancy_map, robot_center_px, 5, (255, 0, 0), -1)
def filter_points_in_radius(all_points, center, radius):
    """
    Lọc các điểm trong bán kính quanh tâm (center)
    all_points: np.ndarray shape (N, 3)
    center: np.array shape (3,)
    radius: float (mm)
    """
    deltas = all_points[:, :2] - center[:2]  # chỉ xét x, y
    distances = np.linalg.norm(deltas, axis=1)
    mask = distances <= radius
    return all_points[mask]

def prune_global_map(global_map, occupancy_probs, map_center_px, resolution,
                     free_threshold=0.3):
    #lọc những điểm nằm ngoài vùng và những điểm có tỉ lệ chiếm dụng nhỏ hơn ngưỡng
    if len(global_map.points) == 0 or occupancy_probs is None:
        return global_map      
    points = np.asarray(global_map.points)
    height, width = occupancy_probs.shape
   
    indices_to_keep = []
    for i, point in enumerate(points):
        px = int(map_center_px[0] + point[0] / resolution)
        py = int(map_center_px[1] - point[1] / resolution)
 
        if not (0 <= px < width and 0 <= py < height):
            indices_to_keep.append(i)
            continue
        if occupancy_probs[py, px] < free_threshold:
            continue
        indices_to_keep.append(i)
 
    return global_map.select_by_index(indices_to_keep)

def filter_new_points_by_occupancy(points_to_add, occupancy_probs, map_center_px, resolution, free_threshold=0.3):
    """Lọc các điểm mới dựa trên bản đồ chiếm dụng hiện có.
    Loại bỏ các điểm rơi vào các ô được coi là không gian trống để giảm nhiễu.
    """
    if len(points_to_add) == 0 or occupancy_probs is None:
        return points_to_add
 
    height, width = occupancy_probs.shape
   
    indices_to_keep = []
    for i, point in enumerate(points_to_add):
        px = int(map_center_px[0] + point[0] / resolution)
        py = int(map_center_px[1] - point[1] / resolution)
 
        if not (0 <= px < width and 0 <= py < height):
            indices_to_keep.append(i)
            continue
 
        if occupancy_probs[py, px] < free_threshold:
            continue
       
        indices_to_keep.append(i)
 
    return points_to_add[indices_to_keep]
def draw_local_map(points, center_pos, map_size=8000, resolution=30):
    """
    Tạo ảnh hiển thị các điểm trong local map
    - center_pos: vị trí robot (x, y)
    - map_size: chiều dài cạnh bản đồ (mm)
    - resolution: mm/pixel
    """
    pixels = int(map_size / resolution)
    map_img = np.full((pixels, pixels, 3), 255, dtype=np.uint8)
    center_px = pixels // 2
 
    for pt in points:
        dx = pt[0] - center_pos[0]
        dy = pt[1] - center_pos[1]
        x_px = int(center_px + dx / resolution)
        y_px = int(center_px - dy / resolution)
        if 0 <= x_px < pixels and 0 <= y_px < pixels:
            cv2.circle(map_img, (x_px, y_px), 1, (0, 0, 0), -1)
 
    return map_img
# --- PHẦN 3: CHƯƠNG TRÌNH CHÍNH ---

if __name__ == "__main__":

    occupancy_map = np.full((Config.MAP_HEIGHT_PIXELS, Config.MAP_WIDTH_PIXELS, 3), 128, dtype=np.uint8)
    map_center_px = (Config.MAP_WIDTH_PIXELS // 2, Config.MAP_HEIGHT_PIXELS // 2)
   
    global_map = o3d.geometry.PointCloud()
    global_pose = np.eye(4)
    current_points_global = np.array([])
    loop_closure_points = [] 


    print("\nBắt đầu xử lý dữ liệu từ file và lập bản đồ. Nhấn 'q' hoặc ESC để thoát.")
    first_scan_points = load_and_prepare_scan(Config.BASE_PATH.format(Config.START_FILE))
    if first_scan_points is None or len(first_scan_points) == 0:
        
        print("Lỗi: Không thể tải scan đầu tiên. Thoát chương trình.")
        exit()

    global_map.points.extend(o3d.utility.Vector3dVector(first_scan_points))
    current_points_global = first_scan_points
    update_occupancy_map(occupancy_map, current_points_global, global_pose[:3, 3], map_center_px, Config.RESOLUTION_MM_PER_PIXEL)
    gg = time.time()
    for i in range(Config.START_FILE + 1, Config.END_FILE):
        print(time.time() - gg)
        gg = time.time()
        scan_file = Config.BASE_PATH.format(i)
        scan_data = load_and_prepare_scan(scan_file)

        if scan_data is None or len(scan_data) == 0:
            print(f"Lỗi tải tệp {scan_file} hoặc tệp rỗng. Bỏ qua.")
            continue
        # print(scan_data)
        current_points = scan_data
        
        if len(current_points) < 10:
            continue
        # Xử lý SLAM
        # Chỉ lấy điểm trong bán kính (ví dụ 3000mm quanh robot)
        robot_xy = global_pose[:3, 3]
        map_points_all = np.asarray(global_map.points)
        map_points_for_icp = filter_points_in_radius(map_points_all, robot_xy, radius=8000.0)

        # # ✅ Vẽ ra ảnh mới
        # local_map_img = draw_local_map(map_points_for_icp, robot_xy, map_size=8000, resolution=Config.RESOLUTION_MM_PER_PIXEL)
        # cv2.imshow("Local Map for ICP", local_map_img)

        map_for_display = occupancy_map.copy()
        tt = time.time()
        rmse, transformation_matrix = gicp(current_points, map_points_for_icp, Config.ICP_THRESHOLD, Config.ICP_VOXEL_SIZE, trans_init=global_pose)
        print(f"{scan_file}ICP RMSE: {rmse:.4f}", time.time() - tt)
        if rmse <= Config.MAX_RMSE1_THRESHOLD:
            global_pose = transformation_matrix # transformation_matrix đã được cộng dồn sau đó lại cập nhật vào global_pose
        if rmse <= Config.MAX_RMSE2_THRESHOLD:
            if len(current_points) == 0:
                continue  # bỏ qua nếu không còn điểm nào
            # Sau khi biến đổi sang tọa độ toàn cục
            current_points_global = transform_points(current_points, global_pose[:3, :3], global_pose[:3, 3])
            
            # Tiếp tục như cũ
            points_to_add = remove_duplicate_points(current_points_global, voxel_size=Config.ICP_VOXEL_SIZE)
            if hasattr(update_occupancy_map, "occupancy_probs"):
               
                points_to_add = filter_new_points_by_occupancy(
                    points_to_add,
                    update_occupancy_map.occupancy_probs,
                    map_center_px,
                    Config.RESOLUTION_MM_PER_PIXEL
                )
            if len(points_to_add) > 0: # thêm danh sách điểm vào danh sach điểm gốc
                global_map.points.extend(o3d.utility.Vector3dVector(points_to_add))
            if len(global_map.points) > 1000: # giảm mẫu
                    global_map = downsample_point_cloud(global_map, Config.ICP_VOXEL_SIZE)

            robot_pos_map = global_pose[:3, 3]  # vị trí của robot trong danh sách điểm gốc
            update_occupancy_map(occupancy_map, current_points_global, robot_pos_map, map_center_px, Config.RESOLUTION_MM_PER_PIXEL) # thêm vào danh sách điểm gốc
            print(update_occupancy_map.occupancy_probs.shape)
            if hasattr(update_occupancy_map, "occupancy_probs"): # kiểm tra xem update_occupancy_map có occupancy_probs hay không
                    # so với bản đồ tỷ lệ để lọc xóa bớt điểm đi (người di chuyển)
                    # print(map_center_px)
                    global_map = prune_global_map(global_map, update_occupancy_map.occupancy_probs, map_center_px, Config.RESOLUTION_MM_PER_PIXEL)
            map_for_display = occupancy_map.copy()

            # vẽ lên ảnh map tỷ lệ
            scan_on_map(map_for_display, current_points_global, map_center_px, Config.RESOLUTION_MM_PER_PIXEL, color=(0, 255, 0))
        # vẽ vị trí và hướng của robot
        draw_robot_pose(map_for_display, global_pose, map_center_px, Config.RESOLUTION_MM_PER_PIXEL, Config.ROBOT_AXIS_LENGTH_MM)
        cv2.imshow("map", map_for_display)

        
        key = cv2.waitKey(1)
        if key == ord('q') or cv2.getWindowProperty('map', cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyAllWindows()