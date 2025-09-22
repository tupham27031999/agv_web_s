import numpy as np
import cv2, threading, time, math, shutil, os
from support_main.lib_main import edit_csv_tab, remove
from support_main import angle_and_distance, gicp_lidar, music
import path
# import crop_img_Atar
# import detect_pallet
# import dich_diem
import webserver
import open3d as o3d
import detect_gicp






path_phan_mem = path.path_phan_mem

# Directories for saving data and maps
SAVED_DATA_DIR = path_phan_mem + "/data_input_output"
PATH_MAPS_DIR = SAVED_DATA_DIR + "/maps"

distan_scan_all = [0, 10000]
path_admin = path_phan_mem + "/setting/admin_window.csv"
if os.name == "nt":
    print("Hệ điều hành là Windows")
    # Đọc file cài đặt cho Windows
    path_admin = path_phan_mem + "/setting/admin_window.csv"
    on_music = 1
elif os.name == "posix":
    print("Hệ điều hành là Ubuntu (Linux)")
    # Đọc file cài đặt cho Ubuntu
    path_admin = path_phan_mem + "/setting/admin_ubuntu.csv"
    on_music = 0
data_admin = edit_csv_tab.load_all_stt(path_admin)
x_goc0 = "none"
y_goc0 = "none"
rotation0 = "none"
kc_them_map = 170
for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "distan_scan_all":
            distan_scan_all = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        if data_admin[i][0] == "x_goc0":
            if data_admin[i][1] != "none":
                x_goc0 = int(float(data_admin[i][1]))
        if data_admin[i][0] == "y_goc0":
            if data_admin[i][1] != "none":
                y_goc0 = int(float(data_admin[i][1]))
        if data_admin[i][0] == "rotation0":
            if data_admin[i][1] != "none":
                rotation0 = float(data_admin[i][1])
        if data_admin[i][0] == "kc_them_map":
            kc_them_map = int(float(data_admin[i][1]))
        if data_admin[i][0] == "anpha_scan":
            anpha_scan = [int(float(data_admin[i][1])), int(float(data_admin[i][2]))] 
        
print("distan_scan_all", distan_scan_all, x_goc0, y_goc0, rotation0, anpha_scan)

class process_data_lidar:
    def __init__(self, window_size,window_size_all,scaling_factor,rmse1,rmse2):
        self.window_size_x = window_size
        self.window_size_y = window_size

        self.window_size_x_img2 = 400
        self.window_size_y_img2 = 400

        self.window_size_x_all = window_size_all
        self.window_size_y_all = window_size_all
        self.scaling_factor = scaling_factor
        self.scan_xy = np.array([[0, 0, 0]])

        self.map_all = np.full((self.window_size_y_all, self.window_size_x_all, 3), (150, 150, 150), np.uint8)
        self.img0 = np.ones((self.window_size_y, self.window_size_x, 3), np.uint8) *150
        self.img1 = self.map_all.copy()
        self.img2 = self.img1.copy()[int(self.window_size_x_all//2 - self.window_size_y_img2):int(self.window_size_x_all//2 + self.window_size_y_img2),
                                     int(self.window_size_x_all//2 - self.window_size_x_img2):int(self.window_size_x_all//2 + self.window_size_x_img2),:]

        self.rmse = 0
        self.rmse1 = rmse1
        self.rmse2 = rmse2

        self.x_goc = int(self.window_size_x_all / 2)
        self.y_goc = int(self.window_size_y_all / 2)
        self.rotation = 0
        self.x_goc_old = self.x_goc
        self.y_goc_old = self.y_goc
        self.tam_x_agv, self.tam_y_agv = [int(self.window_size_x_all / 2), int(self.window_size_y_all / 2)]
        self.huong_x = 0
        self.huong_y = 0

        # self.window_size_img2 = 600
        # self.number_img2 = int(self.window_size_img2/2)
        # self.number_img3 = 20

        self.arr_goc0 = np.array([])
        self.global_map = o3d.geometry.PointCloud()

        self.time_start = 0
        self.update_vi_tri_agv_ban_dau = 1

        # nếu khoảng cách hay góc giữa 2 lần quét liền kề quá lơn sẽ dừng agv
        self.delta_distan = 0
        self.delta_angle = 0

        # bằng 1 thì cập nhật vào map (thực hiện quét map)
        self.add_all_point = 0
        # thêm vào map nếu = 1 ( bỏ qua điều khiện < rmse)
        self.add_map = 0

        self.color_check = [0,0,0]
        self.color = [255,255,255]

        self.sent_data_driver_motor = {"stop": 0, "scan": self.scan_xy, "img1": self.img1, "tam_x_agv": self.tam_x_agv, "tam_y_agv": self.tam_y_agv, 
                                       "huong_agv": self.rotation, "huong_x": self.huong_x, "huong_y": self.huong_y, 
                                       "scaling_factor": self.scaling_factor, "window_size_x_all": self.window_size_x_all, "window_size_y_all": self.window_size_y_all}
        self.vi_tri_agv_new = 0

        self.bien_dem = 0
        self.trung_binh = 0

        self.transformation =np.eye(4)
        self.detect_gicp = detect_gicp.Config()
    def main_loop(self, scan_xy):
        self.process_data_lidar(scan_xy)
        self.load_data_web()
        self.update_vi_tri_agv()
    def load_data_web(self):
        if webserver.dict_chon_ban_do["update"] == 1:
            webserver.dict_chon_ban_do["update"] = 0
            map_name = webserver.dict_chon_ban_do["ten_ban_do"]
            map_path = os.path.join(PATH_MAPS_DIR, map_name)
            if os.path.exists(map_path):
                try:
                    loaded_img = None
                    if map_name.lower().endswith(".npy"):
                        img_array = np.load(map_path)
                        if img_array.ndim == 2: # Grayscale
                            loaded_img = cv2.cvtColor(img_array, cv2.COLOR_GRAY2BGR)
                        elif img_array.ndim == 3:
                            loaded_img = img_array
                        else:
                            print(f"Unsupported .npy array dimension: {img_array.ndim} for {map_name}")

                        if loaded_img is not None and loaded_img.dtype != np.uint8:
                            # Basic scaling for float arrays, assuming range 0-1 or 0-255
                            if loaded_img.max() <= 1.0 and loaded_img.min() >= 0.0:
                                loaded_img = (loaded_img * 255).astype(np.uint8)
                            else:
                                loaded_img = cv2.normalize(loaded_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                    else: # Standard image
                        loaded_img = cv2.imread(map_path)

                    if loaded_img is not None:
                        self.map_all = loaded_img
                except Exception as e:
                    print(f"Exception loading map {map_path}: {e}")
    def create_map(self, px, py):
        self.delta_distan = 0
        self.delta_angle = 0
        for i in range(0, px.shape[0]):
            cv2.line(self.map_all, (int(self.x_goc), int(self.y_goc)), (int(px[i]), int(py[i])), [255,255,255], 1)
            self.map_all[int(py[i]), int(px[i]),:] = self.color_check
    
    
    def update_vi_tri_agv(self):
        # update map từ webserver
        # print("dict_dieu_chinh_vi_tri_agv", webserver.dict_dieu_chinh_vi_tri_agv)
        if webserver.dict_dieu_chinh_vi_tri_agv["update"] == 1 or webserver.dict_dieu_chinh_vi_tri_agv["setup"] == 1:
            if webserver.dict_dieu_chinh_vi_tri_agv["update"] == 1:
                webserver.dict_dieu_chinh_vi_tri_agv["update"] = 0
            x = webserver.dict_dieu_chinh_vi_tri_agv['toa_do_x']
            y = webserver.dict_dieu_chinh_vi_tri_agv['toa_do_y']
            angle = webserver.dict_dieu_chinh_vi_tri_agv['goc_agv']
            self.x_goc = x
            self.y_goc = y
            self.rotation = angle * np.pi / 180
            self.update_vi_tri_agv_ban_dau = 0

    def load_map_lan_dau(self):
        if rotation0 != "none":
            self.rotation = float(rotation0) * np.pi / 180
            webserver.dict_dieu_chinh_vi_tri_agv["goc_agv"] = self.rotation * 180 / np.pi
        else:
            self.rotation = 0
        if x_goc0 != "none":
            self.x_goc = int(float(x_goc0))
            webserver.dict_dieu_chinh_vi_tri_agv["toa_do_x"] = self.x_goc
        else:
            self.x_goc = int(self.window_size_x_all/2)
        if y_goc0 != "none":
            self.y_goc = int(float(y_goc0))
            webserver.dict_dieu_chinh_vi_tri_agv["toa_do_y"] = self.y_goc
        else:
            self.y_goc = int(self.window_size_x_all/2)
        self.map_all = np.full((self.window_size_x_all, self.window_size_x_all, 3), (150, 150, 150), np.uint8)
        self.img0 = np.ones((self.window_size_y, self.window_size_x, 3), np.uint8) * 150
        self.img1 = self.map_all.copy()

        self.update_vi_tri_agv_ban_dau = 0
    def transform_agv_points_to_pixel(self, agv_points_data, rotation_rad, translation_pixel):
        """
        Chuyển đổi các điểm từ hệ tọa độ AGV sang tọa độ pixel trên ảnh,
        bao gồm xoay và tịnh tiến.

        Args:
            agv_points_data (np.ndarray): Mảng NumPy chứa các điểm có dạng [x_agv, y_agv, signal].
                                        x_agv, y_agv là tọa độ của điểm trong hệ AGV.
            rotation_rad (float): Góc xoay (radian) của toàn bộ bản đồ/hệ AGV
                                (xoay quanh gốc (0,0) của hệ AGV).
            translation_pixel (list/tuple): Danh sách hoặc tuple [xt, yt] chứa lượng tịnh tiến
                                            để dịch chuyển gốc AGV (tức là điểm (0,0) trong hệ AGV)
                                            đến vị trí mong muốn trên ảnh.
                                            Lưu ý: xt, yt là khoảng cách pixel từ gốc ảnh đến gốc AGV.
            scaling_factor (float): Hệ số tỉ lệ để chuyển đổi từ đơn vị thực tế (của x_agv, y_agv)
                                    sang đơn vị pixel. (ví dụ: pixel/mét hoặc pixel/mm).
            image_center_x (int): Tọa độ X (pixel) của tâm ảnh.
            image_center_y (int): Tọa độ Y (pixel) của tâm ảnh.

        Returns:
            tuple: Một tuple chứa hai mảng NumPy 1D:
                - px (np.ndarray): Mảng các tọa độ X (pixel) đã xoay và tịnh tiến.
                - py (np.ndarray): Mảng các tọa độ Y (pixel) đã xoay và tịnh tiến.
        """
        if agv_points_data.shape[0] < 1:
            # Nếu không có điểm nào, trả về mảng rỗng
            return np.array([]), np.array([])

        # 1. Tách các thành phần x_agv, y_agv
        x_agv_points = agv_points_data[:, 0]
        y_agv_points = agv_points_data[:, 1]
        # signals = agv_points_data[:, 2] # signal không cần cho tính toán px, py

        # 2. Áp dụng phép xoay quanh gốc (0,0) của hệ AGV
        # Pre-calculate sin/cos để tối ưu
        cos_rot = np.cos(rotation_rad)
        sin_rot = np.sin(rotation_rad)

        x_rotated = x_agv_points * cos_rot + y_agv_points * sin_rot
        y_rotated = x_agv_points * sin_rot - y_agv_points * cos_rot

        # 3. Áp dụng scaling_factor và tịnh tiến (translation) để chuyển sang tọa độ pixel
        # Nếu translation_pixel[0] và translation_pixel[1] đã là tọa độ pixel của gốc AGV (0,0) trên ảnh,
        # và ảnh có trục Y dương đi xuống:
        px = translation_pixel[0] + x_rotated
        py = translation_pixel[1] - y_rotated # Đảo ngược trục Y
        return px, py

    def process_data_lidar(self, scan_xy):
        if distan_scan_all[0] == 1:
            # Lấy cột x và y
            x_coords = scan_xy[:, 0]
            y_coords = scan_xy[:, 1]

            # Tính khoảng cách từ (x, y) đến (0, 0)
            distances = np.sqrt(x_coords**2 + y_coords**2)

            # Lọc các hàng mà khoảng cách < 1500
            scan_all = scan_xy[distances < distan_scan_all[1]]
        else:
            scan_all = scan_xy

        self.scan_xy = scan_all
        self.sent_data_driver_motor["scan"] = self.scan_xy

        

        if self.scan_xy.shape[0] > 1:
            if self.time_start == 0:
                self.time_start = time.time()
            self.img1 = self.map_all.copy()
            self.sent_data_driver_motor["img1"] = self.img1.copy()

            px0, py0 = self.transform_agv_points_to_pixel(self.scan_xy, 0, [0, 0])

            if self.update_vi_tri_agv_ban_dau == 1 and time.time() - self.time_start > 1 and len(px0) > 60:
                # self.load_map_lan_dau()
                # self.detect_gicp.detect(self.scan_xy)
                # self.map_all = detect_gicp.update_occupancy_map(self.map_all, self.scan_xy, self.transformation[:3, 3], [self.window_size_x_all//2, self.window_size_y_all//2])
                self.update_vi_tri_agv_ban_dau = 0
            if self.update_vi_tri_agv_ban_dau == 0 and len(px0) > 60:
                arr_test = np.vstack((px0, py0, np.zeros_like(px0))).T  # Thêm chiều z = 0 để tạo PointCloud 3D
                
                if webserver.dict_dieu_chinh_vi_tri_agv["setup"] == 0:
                    stop_agv = 0
                    tt = time.time()
                    rmse, self.transformation = self.detect_gicp.detect(self.scan_xy, MAX_RMSE_THRESHOLD = self.rmse1)
                    print(time.time() - tt)
                    r = self.transformation[:3, :3]
                    t = self.transformation[:3, 3]
                    self.rmse = rmse

                    if rmse <= self.rmse1:
                        # self.rotation = np.arcsin(r[0, 1])
                        self.rotation = -np.arctan2(self.transformation[1, 0], self.transformation[0, 0])
                        # yaw_deg = np.degrees(yaw_rad)
                        self.x_goc = t[0] * self.scaling_factor + self.window_size_x_all//2
                        self.y_goc = t[1] * self.scaling_factor + self.window_size_y_all//2
                        self.x_goc_old = self.x_goc
                        self.y_goc_old = self.y_goc
                        h_img1,w_img1,_ = self.img1.shape

                        # chuyen arr_test ve vi tri self.arr_goc (test ngay lúc trc)
                        new_arr_ok = gicp_lidar.transform_points(arr_test, r, t) 
                        new_arr_ok[:, 0] = new_arr_ok[:, 0] * self.scaling_factor + self.window_size_x_all//2
                        new_arr_ok[:, 1] = new_arr_ok[:, 1] * self.scaling_factor + self.window_size_y_all//2

                        for ii in range(0, max(len(new_arr_ok), new_arr_ok.shape[0])):
                            # hiển thị kết quả của icp lên ảnh
                            if ii < len(new_arr_ok[:, 0]) - 1 and int(new_arr_ok[ii, 0]) < w_img1 and int(new_arr_ok[ii, 1]) < h_img1:
                                cv2.circle(self.img1, (int(new_arr_ok[ii, 0]), int(new_arr_ok[ii, 1])), 1, (255, 0, 255), -1)
                            # cập nhật vào map all nếu cần
                            if ii < len(new_arr_ok[:, 0]) - 1 and ii < len(new_arr_ok[:, 1]) - 1 and self.add_all_point == 1:
                                # if (rmse < self.rmse2 or self.add_map == 1) and int(new_arr_ok[ii, 1]) < self.window_size_y_all and int(new_arr_ok[ii, 1]) > 0 and \
                                if int(new_arr_ok[ii, 1]) < self.window_size_y_all and int(new_arr_ok[ii, 1]) > 0 and \
                                                int(new_arr_ok[ii, 0]) < self.window_size_x_all and int(new_arr_ok[ii, 0]) > 0:
                                    if (self.map_all[int(new_arr_ok[ii, 1]), int(new_arr_ok[ii, 0]), :3].tolist() != [255,255,255] or self.add_map == 1):
                                        # nếu điểm đó đang khác màu trắng  [255,255,255]
                                        if (self.map_all[int(new_arr_ok[ii, 1]), int(new_arr_ok[ii, 0]), :3].tolist() != [0, 0, 0]):
                                            # nếu điểm đó khác màu đen -- > màu xám
                                            # cần kiểm tra trên đường đi điểm màu đen nào cách điểm đang xét > step không nếu có thì chuyển điểm đó màu trắng
                                            # không thêm điểm nếu đường quét có số điểm màu đen lớn hơn number
                                            self.bresenham_line(self.map_all, int(self.x_goc), int(self.y_goc), 
                                                                int(new_arr_ok[ii, 0]), int(new_arr_ok[ii, 1]),  
                                                                step = 20, color=self.color, color_check=self.color_check, number = 1)
                                    else:
                                        # nếu điểm đó đang là màu trắng
                                        self.bresenham_distan(self.map_all, int(self.x_goc), int(self.y_goc),
                                                            int(new_arr_ok[ii, 0]), int(new_arr_ok[ii, 1]),
                                                            4, color_check=self.color_check)
                        # self.map_all = detect_gicp.update_occupancy_map(self.map_all, self.scan_xy, self.transformation[:3, 3], [self.window_size_x_all//2, self.window_size_y_all//2])
                        self.add_map = 0
                    else:
                        new_arr_ok = arr_test
                        new_arr_ok[:, 0] = new_arr_ok[:, 0] * self.scaling_factor + self.window_size_x_all//2
                        new_arr_ok[:, 1] = new_arr_ok[:, 1] * self.scaling_factor + self.window_size_y_all//2
                        stop_agv = 1
                        

                    self.rotation = self.normalize_angle_rad(self.rotation)

                    self.tam_x_agv, self.tam_y_agv = self.translate_point(self.x_goc, self.y_goc, - self.rotation, distance=0)

                else:
                    stop_agv = 1
                    new_arr_ok = arr_test
                    new_arr_ok[:, 0] = new_arr_ok[:, 0] * self.scaling_factor + self.window_size_x_all//2
                    new_arr_ok[:, 1] = new_arr_ok[:, 1] * self.scaling_factor + self.window_size_y_all//2
                    h_img1,w_img1,_ = self.img1.shape
                    self.tam_x_agv = self.x_goc
                    self.tam_y_agv = self.y_goc
                    for ii in range(0, max(len(new_arr_ok), new_arr_ok.shape[0])):
                        if ii < len(new_arr_ok[:, 0]) - 1 and int(new_arr_ok[ii, 0]) < w_img1 and int(new_arr_ok[ii, 1]) < h_img1:
                            cv2.circle(self.img1, (int(new_arr_ok[ii, 0]), int(new_arr_ok[ii, 1])), 1, (0, 255, 0), -1)
                    
                cv2.circle(self.img1, (int(self.tam_x_agv), int(self.tam_y_agv)), 5, (255, 0, 0), -1)
                self.huong_x = int(self.tam_x_agv + 20 * math.cos(np.pi - self.rotation))
                self.huong_y = int(self.tam_y_agv + 20 * math.sin(np.pi - self.rotation))

                self.huong_x2 = int(self.tam_x_agv + 20 * math.cos(-self.rotation))
                self.huong_y2 = int(self.tam_y_agv + 20 * math.sin(-self.rotation))

                cv2.arrowedLine(self.img1, (int(self.tam_x_agv), int(self.tam_y_agv)), (self.huong_x2, self.huong_y2), (255, 0, 0), 1, tipLength=0.2)
                if self.x_goc - self.window_size_x >=0 and self.x_goc + self.window_size_x < self.window_size_x_all and \
                                    self.y_goc - self.window_size_y >=0 and self.y_goc + self.window_size_y < self.window_size_y_all:
                    self.img2 = self.img1.copy()[int(self.y_goc - self.window_size_y_img2):int(self.y_goc + self.window_size_y_img2),
                                                 int(self.x_goc - self.window_size_x_img2):int(self.x_goc + self.window_size_x_img2),:]
                
                webserver.points_color_blue = new_arr_ok[:,:2]
                if webserver.dict_dieu_chinh_vi_tri_agv["update"] == 0 and webserver.dict_dieu_chinh_vi_tri_agv["setup"] == 0:
                    webserver.dict_dieu_chinh_vi_tri_agv["toa_do_x"] = self.tam_x_agv
                    webserver.dict_dieu_chinh_vi_tri_agv["toa_do_y"] = self.tam_y_agv
                    webserver.dict_dieu_chinh_vi_tri_agv["goc_agv"] = (self.rotation * 180 / np.pi) % 360
                self.sent_data_driver_motor["tam_x_agv"] = self.tam_x_agv
                self.sent_data_driver_motor["tam_y_agv"] = self.tam_y_agv
                self.sent_data_driver_motor["rotation"] = self.rotation
                self.sent_data_driver_motor["stop"] = stop_agv
                self.sent_data_driver_motor["huong_x"] = self.huong_x
                self.sent_data_driver_motor["huong_y"] = self.huong_y
    def translate_point(self, x, y, alpha, distance=5):
        """
        Dịch chuyển điểm (x, y) theo góc alpha và khoảng cách distance.
        
        Args:
            x (float): Tọa độ x của điểm đầu vào.
            y (float): Tọa độ y của điểm đầu vào.
            alpha (float): Góc alpha (đơn vị: radians).
            distance (float): Khoảng cách dịch chuyển (mặc định: 5 pixel).
        
        Returns:
            tuple: Tọa độ điểm mới (x_new, y_new).
        """
        x_new = x + distance * math.cos(alpha)
        y_new = y + distance * math.sin(alpha)
        return x_new, y_new
    def normalize_angle_rad(self, angle):
        """
        Chuyển đổi góc (radian) về khoảng [-pi, pi] radian.
        Ví dụ: 7 -> ~0.716 rad, -4 -> ~2.283 rad
        """
        angle = angle % (2 * np.pi)
        if angle > np.pi:
            angle -= 2 * np.pi
        if angle < -np.pi:
            angle += 2 * np.pi
        return angle    
    
    def find_opposite_point(self, x0, y0, x1, y1):
        x2 = 2 * x1 - x0
        y2 = 2 * y1 - y0
        return x2, y2
    
    def bresenham_distan(self,image, x0, y0, x1, y1, number, color_check = [0,0,0]):
        x_tg, y_tg = self.find_opposite_point(x0, y0, x1, y1) # điểm đối xứng với x1,y1 qua x0,y0 (x_goc, y_goc)

        x0 = x1 # điểm đích
        y0 = y1
        x1 = x_tg
        y1 = y_tg
        # kiem tra xem tu x0, y0 den x1, y1 co diem mau den nao trong khoang cach distan hay khong, co thi True, khong thi False
        xa = x0
        ya = y0
        
        dx = abs(x1 - xa)
        dy = abs(y1 - ya)
        sx = 1 if xa < x1 else -1
        sy = 1 if ya < y1 else -1
        err = dx - dy
        bien_dem = 0
        ok1 = False
        ok2 = True
        # kiểm tra điều kiện có vẽ thêm điểm không, nếu tường quá dày thì không vẽ thêm nữa ( > 4) càng lớn tường càng dày
        while True:
            if bien_dem <= number:
                if bien_dem <= int(number/2):
                    if ok1 == False:
                        if image[ya, xa, :].tolist() == color_check:
                            ok1 = True
                else:
                    if ok2 == True:
                        if image[ya, xa, :].tolist() == color_check:
                            ok2 = False
            else:
                if ok1 == True and ok2 == True: # tường mỏng thì thêm điểm
                    image[y0, x0, :] = color_check
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                xa += sx
            if e2 < dx:
                err += dx
                ya += sy
            bien_dem = bien_dem + 1
    def bresenham_line(self, image, x0, y0, x1, y1, step, color=[255, 255, 255], color_check = [0, 0, 0], number = 2): #255, 255, 255 màu trắng
        '''
            color=[255, 255, 255, 255]: màu trắng
            color_check = [0, 0, 0, 255]: màu đen
            step: tính khoảng cách từ điểm màu đen đến điểm quét tiếp theo màu đên nếu quá step điểm đen 1 sẽ chuyển màu trắng   
            number: độ dày của tường nếu quá number thì sẽ không thêm điểm đen mới nữa, và thoát vòng lặp

        '''
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        so_lan = 0
        start = 0
        bien_dem = 0
        point_check = []
        
        # khoảng cách thêm điểm vào trong map moi
        distan2 = angle_and_distance.calculate_distance([self.x_goc,self.y_goc],[x1,y1])
        
        while distan2 <= kc_them_map:

            if image[y0, x0, :].tolist() == color_check: # điểm đang xét là màu đen
                start = 1
                point_check = [x0,y0]
                bien_dem = 0
                so_lan = so_lan + 1
            else:
                image[y0, x0, :] = color # nếu điểm đang xét không phải màu đen thì chuyển màu trắng
            if start == 1: # khi bắt đầu có điểm màu đen thực hiện tăng biến đếm
                bien_dem = bien_dem + 1
            if bien_dem == step: # điểm xét lúc trước nếu biến đếm lên đến step thì sẽ chuyển màu trắng
                # image[point_check[1], point_check[0], :] = color
                point_check = []

            # if so_lan >= number: # không thêm điểm mới nếu quá 2 so lần gặp điểm đen là 2
            #     break
            
            
            if x0 == x1 and y0 == y1:
                image[y1, x1, :] = color_check
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy  
        return image
    def save_current_map(self, map_name, save_path_dir):
        """
        Lưu self.map_all (3 kênh BGR) vào file .npy.
        Args:
            map_name (str): Tên của bản đồ (không bao gồm phần mở rộng .npy).
            save_path_dir (str): Đường dẫn đến thư mục để lưu bản đồ.
        """
        if not map_name:
            print("Lỗi: Tên bản đồ rỗng. Không thể lưu.")
            return
        if not os.path.exists(save_path_dir):
            try:
                os.makedirs(save_path_dir, exist_ok=True)
                print(f"Đã tạo thư mục: {save_path_dir}")
            except OSError as e:
                print(f"Lỗi tạo thư mục {save_path_dir}: {e}")
                return

        file_path = os.path.join(save_path_dir, f"{map_name}.npy")
        
        if self.map_all.shape[2] >= 3: # Đảm bảo có ít nhất 3 kênh
            map_to_save = self.map_all # Lấy 3 kênh đầu tiên (giả sử là BGR)
        else:
            print(f"Lỗi: map_all không có đủ 3 kênh màu: {self.map_all.shape}")
            return
        try:
            np.save(file_path, map_to_save)
            print(f"Bản đồ đã được lưu thành công vào: {file_path}")
        except Exception as e:
            print(f"Lỗi khi lưu bản đồ vào {file_path}: {e}")