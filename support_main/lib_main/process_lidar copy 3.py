import numpy as np
import cv2, threading, time, math, shutil, os
from support_main.lib_main import edit_csv_tab, remove
from support_main import angle_and_distance, gicp_lidar, tim_duong_di, music
import path
# import crop_img_Atar
# import detect_pallet
# import dich_diem
import webserver
from skimage.draw import line
import open3d as o3d






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
        print("img0.shape", window_size)
        self.window_size_x = window_size
        self.window_size_y = window_size

        self.window_size_x_img2 = 400
        self.window_size_y_img2 = 400

        self.window_size_x_all = window_size_all
        self.window_size_y_all = window_size_all
        self.scaling_factor = scaling_factor
        self.scan_xy = np.array([[0, 0, 0]])

        self.map_all = np.full((self.window_size_y_all, self.window_size_x_all, 3), (150, 150, 150), np.uint8)
        self.map_all_2d = np.zeros((self.window_size_y_all, self.window_size_x_all), dtype=np.float32)
        self.img0 = np.ones((self.window_size_y, self.window_size_x, 3), np.uint8) *150
        self.img1 = self.map_all.copy()
        self.img2 = self.map_all.copy()[int(self.window_size_x_all//2 - self.window_size_y_img2):int(self.window_size_x_all//2 + self.window_size_y_img2),
                                     int(self.window_size_x_all//2 - self.window_size_x_img2):int(self.window_size_x_all//2 + self.window_size_x_img2)]

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



        self.trans_init = np.eye(4)
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
    
    def process_data_lidar(self, scan_xy):
        if distan_scan_all[0] == 1:
            x_coords = scan_xy[:, 0]
            y_coords = scan_xy[:, 1]
            distances = np.sqrt(x_coords**2 + y_coords**2)
            scan_all = scan_xy[distances < distan_scan_all[1]]
        else:
            scan_all = scan_xy
        self.scan_xy = scan_all
        self.sent_data_driver_motor["scan"] = scan_xy

        if webserver.dict_dieu_chinh_vi_tri_agv["setup"] == 0:
            px = self.scan_xy[:,0]
            py = self.scan_xy[:,1]
            arr_test = np.vstack((px, py, np.zeros_like(px))).T  # Thêm chiều z = 0 để tạo PointCloud 3D
            print(arr_test)
            if self.scan_xy.shape[0] > 50:
                if self.update_vi_tri_agv_ban_dau == 1:
                    self.rmse, self.trans_init = gicp_lidar.gicp(arr_test, arr_test, trans_init=self.trans_init, threshold=200, max_iteration=200, voxel_size=10)
                    self.update_vi_tri_agv_ban_dau = 0
                else:
                    self.rmse, self.trans_init = gicp_lidar.gicp(arr_test, self.arr_goc0, trans_init=self.trans_init, threshold=200, max_iteration=200, voxel_size=10)
                add_arr_goc0 = gicp_lidar.transform_points(arr_test, self.trans_init[:3, :3], self.trans_init[:3, 3])
                self.arr_goc0.points.extend(o3d.utility.Vector3dVector(add_arr_goc0))
                print(self.rmse)
        else:
            stop_agv = 1
            new_arr_ok = arr_test
            self.tam_x_agv = self.x_goc
            self.tam_y_agv = self.y_goc

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
        # tìm điểm đối xứng với x0,y0 qua x1,y1
        x2 = 2 * x1 - x0
        y2 = 2 * y1 - y0
        return x2, y2
    
    def bresenham_distan(self,image, x0, y0, x1, y1, number, color_check = [0,0,0]):
        '''
            O(x0, y0)
            A0(x1,y1)
            B0(x_tg,y_tg) đối xứng với O qua A0
            A1 thuộc OA0 và A0A1 = number/2
            A2 thuộc A0B0 và A0A2 = number/2
        '''
        # điểm B0
        x_tg, y_tg = self.find_opposite_point(x0, y0, x1, y1) # điểm đối xứng với x0,y0  (x_goc, y_goc) qua x1,y1


        # kiem tra xem tu x0, y0 den x1, y1 co diem mau den nao trong khoang cach distan hay khong, co thi True, khong thi False

        # điểm A0
        xa = x1
        ya = y1
        # điểm B0
        x1 = x_tg
        y1 = y_tg
        # điểm từ A0 đến B0
        x0 = xa # coi như là x0 của bresenham_line
        y0 = ya
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        bien_dem = 0
        
        ok1 = False
        ok2 = True
        # kiểm tra điều kiện có vẽ thêm điểm không, nếu tường quá dày thì không vẽ thêm nữa ( > 4) càng lớn tường càng dày
        # điểm 
        while True:
            if bien_dem <= number:
                if bien_dem <= int(number/2):
                    # duyệt từ A0 đến B0 nếu có điểm màu đen thì ok1 == True
                    if ok1 == False:
                        if image[y0, x0, :].tolist() == color_check:
                            ok1 = True
                else:
                    # duyệt từ A0 đến B0 nếu có điểm màu đen thì ok2 == False (sau khi có điểm màu đen)
                    if ok2 == True:
                        if image[y0, x0, :].tolist() == color_check:
                            ok2 = False
            else:
                if ok1 == True and ok2 == True: # tường mỏng thì thêm điểm
                    image[ya, xa, :] = color_check
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
            bien_dem = bien_dem + 1

    # def bresenham_line(x0, y0, x1, y1):
    #     """
    #     Tạo danh sách các điểm (x, y) trên đường từ (x0, y0) đến (x1, y1) bằng thuật toán Bresenham.
    #     Trả về: danh sách các điểm pixel (x, y), bao gồm cả điểm đầu và cuối.
    #     """
    #     x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    #     points = []

    #     dx = abs(x1 - x0)
    #     dy = abs(y1 - y0)
    #     x, y = x0, y0
    #     sx = -1 if x0 > x1 else 1
    #     sy = -1 if y0 > y1 else 1

    #     if dx > dy:
    #         err = dx / 2.0
    #         while x != x1:
    #             points.append((x, y))
    #             err -= dy
    #             if err < 0:
    #                 y += sy
    #                 err += dx
    #             x += sx
    #     else:
    #         err = dy / 2.0
    #         while y != y1:
    #             points.append((x, y))
    #             err -= dx
    #             if err < 0:
    #                 x += sx
    #                 err += dy
    #             y += sy
    #     points.append((x1, y1))  # điểm cuối (occupied)
    #     return points

    # def bresenham_line(self, image, x0, y0, x1, y1, step, color=[255, 255, 255, 255], color_check = [0, 0, 0, 255], number_1 = 3): #255, 255, 255 màu trắng
    #     '''
    #         duyện từ x0,y0 đến x1,y1
    #         color=[255, 255, 255, 255]: màu trắng
    #         color_check = [0, 0, 0, 255]: màu đen
    #         step: tính khoảng cách từ điểm màu đen đến điểm quét tiếp theo màu đên nếu quá step điểm đen 1 sẽ chuyển màu trắng   
    #         number: độ dày của tường nếu quá number thì sẽ không thêm điểm đen mới nữa, và thoát vòng lặp

    #     '''

    #     dx = abs(x1 - x0)
    #     dy = abs(y1 - y0)
    #     sx = 1 if x0 < x1 else -1
    #     sy = 1 if y0 < y1 else -1
    #     err = dx - dy
    #     start_diem_den = 0
    #     so_diem_khong_den = 0
    #     ds_diem_den = []
    #     trang_chuyen_den = []
        
    #     # khoảng cách thêm điểm vào trong map moi
    #     distan2 = angle_and_distance.calculate_distance([self.x_goc,self.y_goc],[x1,y1])
        
    #     while distan2 <= kc_them_map:
    #         if image[y0, x0, :].tolist() == color_check:
    #             distan = tim_duong_di.calculate_distance([x0,y0],[x1,y1])
    #             if distan > step:
    #                 image[y0, x0, :] = color 
    #             else:
    #                 ds_diem_den.append([x0, y0])
    #         else:
    #             image[y0, x0, :] = color 

    #         if len(ds_diem_den) >= number_1: # không thêm điểm mới nếu quá 2 so lần gặp điểm đen là 2
    #             break

    #         if x0 == x1 and y0 == y1:
    #             image[y1, x1, :] = color_check
    #             break
    #         e2 = err * 2
    #         if e2 > -dy:
    #             err -= dy
    #             x0 += sx
    #         if e2 < dx:
    #             err += dx
    #             y0 += sy  
    #     return image
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