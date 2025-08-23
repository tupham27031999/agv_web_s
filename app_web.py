from flask import Flask, render_template, url_for, Response, request, jsonify
import numpy as np
import cv2
import math
import threading


height = 2000
width = 2000

image_all = np.ones((height, width, 3), dtype=np.uint8) * 100
image_all_web = image_all.copy()

# khai báo
list_van_toc = [0.1, 0.2, 0.3, 0.4, 0.5]
list_van_toc_re = [5, 10, 15, 20]
danh_sach_ban_do = ["Bản đồ 1", "Bản đồ 2", "Bản đồ 3", "Bản đồ 4", "Bản đồ 5"]
danh_sach_khu_vuc = ["Khu vực 1", "Khu vực 2", "Khu vực 3", "Khu vực 4", "Khu vực 5"]

# output web
dict_cai_dat = {
    "update": 0,
    "tien_max": 0.2,  # Tốc độ tối đa tiến
    "re_max": 10,  # Tốc độ tối đa rẽ
    }
# AGV position adjustment
dict_dieu_chinh_vi_tri_agv = {"toa_do_x": 1000, "toa_do_y": 1000, "goc_agv": 0, "setup": 0, "update": 0}
points_color_blue = np.array([[200, 200], [300, 300], [400, 400]])  # Example blue points
points_color_red = np.array([[600, 600], [700, 700], [800, 800]])  # Example red points
image_lock = threading.Lock()

# điều khiển quét bản đồ mới hoặc cập nhật bản đồ
dict_ban_do_moi = {
    "ten_ban_do_cu": "",
    "tai_ban_do_cu": 0,
    "cap_nhat_ban_do_cu":0, # nếu == 1 thì bản đồ hiện tại sẽ cập nhật thêm

    "tao_ban_do_moi": 0,
    "ten_ban_do_moi": "",
    "luu_ban_do_moi": 0,

    "dieu_khien_thu_cong": 0, # chỉ bằng 1 khi nhấn nút (điều khiển thủ công và (cap_nhat_ban_do_cu == 1 hoặc tao_ban_do_moi == 1)
    "tien": 0, # bằng 1 khi nhấn nút mũi tên lên
    "lui": 0, # bằng 1 khi nhấn nút mũi tên lùi
    "re_trai": 0,
    "re_phai": 0
}
# Khởi tạo ứng dụng Flask
app = Flask(__name__)

# Định nghĩa route cho trang chủ
@app.route('/')
def home():
    # Sử dụng template 'index.html'
    return render_template(
        'index.html', 
        title='Trang Chủ',
        list_van_toc=list_van_toc,
        list_van_toc_re=list_van_toc_re,
        danh_sach_ban_do=danh_sach_ban_do,
        danh_sach_khu_vuc=danh_sach_khu_vuc,
        dict_cai_dat=dict_cai_dat,
    )


# Định nghĩa route để hiển thị ảnh
@app.route('/img_none_all')
def img_none_all():
    # Mã hóa ảnh từ numpy array sang định dạng JPEG
    # app.image_all được gán từ main.py
    success, encoded_image = cv2.imencode('.jpg', image_all)
    if not success:
        return "Lỗi mã hóa ảnh", 500
    # Tạo một response với dữ liệu ảnh và mimetype phù hợp
    return Response(encoded_image.tobytes(), mimetype='image/jpeg')


# Route để cập nhật cài đặt từ web
@app.route("/update_setting", methods=['POST'])
def update_setting():
    global dict_cai_dat
    data = request.json
    
    # Lấy các giá trị từ payload JSON
    tien_max_new = data.get('tien_max')
    re_max_new = data.get('re_max')
    update_flag = data.get('update')

    # Cập nhật giá trị nếu chúng tồn tại trong payload
    if tien_max_new is not None:
        dict_cai_dat['tien_max'] = tien_max_new
        print(f"Giá trị 'tien_max' đã được cập nhật thành: {tien_max_new}")

    if re_max_new is not None:
        dict_cai_dat['re_max'] = re_max_new
        print(f"Giá trị 're_max' đã được cập nhật thành: {re_max_new}")
    
    # Cập nhật cờ nếu nó tồn tại
    if update_flag is not None:
        dict_cai_dat['update'] = update_flag
        print("dict_cai_dat[\"update\"] đã được đặt thành 1")

    print("dict_cai_dat: ", dict_cai_dat)

    return jsonify({"status": "success", "message": "Settings updated successfully."})
    
# Route để xử lý yêu cầu tải lên bản đồ mới
@app.route('/upload_map', methods=['POST'])
def upload_map():
    """
    Nhận yêu cầu từ client để cập nhật image_all dựa trên bản đồ được chọn.
    """
    global image_all, image_lock
    data = request.get_json()
    map_name = data.get('map_name')

    if not map_name:
        return jsonify({"status": "error", "message": "Tên bản đồ không được cung cấp"}), 400

    with image_lock:
        # --- LOGIC CẬP NHẬT BẢN ĐỒ THỰC TẾ ---
        # Đây là nơi bạn sẽ đặt logic tải bản đồ mới từ file, ví dụ:
        # image_all = cv2.imread(f"path/to/maps/{map_name}.png")
        #
        # Để minh họa rằng việc cập nhật đã hoạt động, tôi sẽ tạo một ảnh mới
        # và vẽ tên bản đồ đã chọn lên đó.
        # Bạn hãy thay thế phần minh họa này bằng logic tải file ảnh thực tế của bạn.
        new_image = np.ones((height, width, 3), dtype=np.uint8) * 240 # Tạo nền màu trắng xám
        cv2.putText(new_image,
                    f"Da tai: {map_name}",
                    (50, 150), # Vị trí text
                    cv2.FONT_HERSHEY_SIMPLEX,
                    4, (0, 100, 0), 5, cv2.LINE_AA) # Màu, kích thước, độ dày
        image_all = new_image.copy() # Gán ảnh mới vào biến toàn cục image_all

    # Trả về thông báo thành công
    return jsonify({"status": "success", "message": f"Đã cập nhật bản đồ thành {map_name}"})

# Endpoint để client lấy thông tin vị trí AGV
@app.route('/get_agv_state', methods=['GET'])
def get_agv_state_route():
    global dict_dieu_chinh_vi_tri_agv, points_color_blue, points_color_red, image_lock
    agv_body_coords_list = []
    agv_arrow_coords_list = []
    blue_points_list = []
    red_points_list = []
    with image_lock: # Đảm bảo an toàn luồng khi đọc các biến này
        center_x_map = float(dict_dieu_chinh_vi_tri_agv['toa_do_x'])
        center_y_map = float(dict_dieu_chinh_vi_tri_agv['toa_do_y'])
        angle_deg = - float(dict_dieu_chinh_vi_tri_agv['goc_agv'])
        angle_rad = math.radians(angle_deg)

        # AGV Body (Rectangle) - Define local coords then rotate and translate
        rect_width_local = 30  # pixels
        rect_height_local = 20 # pixels
        
        hw = rect_width_local / 2 # half-width
        hh = rect_height_local / 2 # half-height
        local_body_corners_np  = np.array([
            [-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]
        ], dtype=np.float32) # Use float32 for cv2.pointPolygonTest


        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        rotation_m = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

        rotated_local_body_corners = np.dot(local_body_corners_np, rotation_m.T)
        final_body_corners_map_space = rotated_local_body_corners + np.array([center_x_map, center_y_map])

        # Rasterize the AGV body rectangle
        min_x_body = int(np.floor(np.min(final_body_corners_map_space[:, 0])))
        max_x_body = int(np.ceil(np.max(final_body_corners_map_space[:, 0])))
        min_y_body = int(np.floor(np.min(final_body_corners_map_space[:, 1])))
        max_y_body = int(np.ceil(np.max(final_body_corners_map_space[:, 1])))

        # The contour for pointPolygonTest needs to be of type int32 or float32.
        # final_body_corners_map_space is already float32 due to local_body_corners_np.
        contour_body_for_test = final_body_corners_map_space.astype(np.float32) 

        temp_body_coords_list = []
        for x_coord_test in range(min_x_body, max_x_body + 1):
            for y_coord_test in range(min_y_body, max_y_body + 1):
                if cv2.pointPolygonTest(contour_body_for_test, (float(x_coord_test), float(y_coord_test)), False) >= 0:
                    temp_body_coords_list.append([x_coord_test, y_coord_test])
        agv_body_coords_list = temp_body_coords_list

        # AGV Arrow (Triangle) - Define local coords then rotate and translate
        arrow_shaft_len_local = 25 
        arrow_head_len_local = 10
        arrow_head_width_local = 12

        p_tip_local = np.array([arrow_shaft_len_local + arrow_head_len_local, 0])
        p_base1_local = np.array([arrow_shaft_len_local, -arrow_head_width_local / 2])
        p_base2_local = np.array([arrow_shaft_len_local, arrow_head_width_local / 2])
        local_arrow_vertices_np = np.array([p_tip_local, p_base1_local, p_base2_local], dtype=np.float32)

        rotated_local_arrow_vertices = np.dot(local_arrow_vertices_np, rotation_m.T)
        final_arrow_vertices_map_space = rotated_local_arrow_vertices + np.array([center_x_map, center_y_map])

        # Rasterize the AGV arrow triangle
        min_x_arrow = int(np.floor(np.min(final_arrow_vertices_map_space[:, 0])))
        max_x_arrow = int(np.ceil(np.max(final_arrow_vertices_map_space[:, 0])))
        min_y_arrow = int(np.floor(np.min(final_arrow_vertices_map_space[:, 1])))
        max_y_arrow = int(np.ceil(np.max(final_arrow_vertices_map_space[:, 1])))

        contour_arrow_for_test = final_arrow_vertices_map_space.astype(np.float32)

        temp_arrow_coords_list = []
        for x_coord_test in range(min_x_arrow, max_x_arrow + 1):
            for y_coord_test in range(min_y_arrow, max_y_arrow + 1):
                if cv2.pointPolygonTest(contour_arrow_for_test, (float(x_coord_test), float(y_coord_test)), False) >= 0:
                    temp_arrow_coords_list.append([x_coord_test, y_coord_test])
        agv_arrow_coords_list = temp_arrow_coords_list

        # Chuyển đổi NumPy arrays thành list để có thể serialize JSON
        if isinstance(points_color_blue, np.ndarray):
            blue_points_list = points_color_blue.tolist()
        else: # Nếu là list, đảm bảo nó là list các list (cho JSON)
            blue_points_list = [list(p) for p in points_color_blue] if points_color_blue else []

        if isinstance(points_color_red, np.ndarray):
            red_points_list = points_color_red.tolist()
        else: # Nếu là list, đảm bảo nó là list các list
            red_points_list = [list(p) for p in points_color_red] if points_color_red else []

    return jsonify({
        "status": "success",
        "agv_body_coords": agv_body_coords_list,
        "agv_arrow_coords": agv_arrow_coords_list,
        "points_blue": blue_points_list,
        "points_red": red_points_list
    }), 200
@app.route('/update_map_status', methods=['POST'])
def update_map_status():
    global dict_ban_do_moi, danh_sach_ban_do
    data = request.json
    action = data.get('action')
    status = data.get('status')
    map_name = data.get('map_name')

    print(f"Nhận yêu cầu: {action}, Trạng thái: {status}, Tên bản đồ: {map_name}")

    if action == 'load_old_map':
        dict_ban_do_moi["ten_ban_do_cu"] = map_name
        dict_ban_do_moi["tai_ban_do_cu"] = 1
        return jsonify({"status": "success", "message": "Đã đặt cờ tải bản đồ cũ"})

    elif action == 'update_old_map':
        dict_ban_do_moi["cap_nhat_ban_do_cu"] = status
        return jsonify({"status": "success", "message": "Đã cập nhật cờ cập nhật bản đồ cũ"})

    elif action == 'create_new_map':
        dict_ban_do_moi["tao_ban_do_moi"] = 1
        return jsonify({"status": "success", "message": "Đã đặt cờ tạo bản đồ mới"})

    elif action == 'update_new_map_name':
        dict_ban_do_moi["ten_ban_do_moi"] = map_name
        return jsonify({"status": "success", "message": "Đã cập nhật tên bản đồ mới"})

    elif action == 'save_new_map':
        if dict_ban_do_moi["ten_ban_do_moi"] != "":
            dict_ban_do_moi["luu_ban_do_moi"] = 1
            # Thêm tên bản đồ mới vào danh sách (để nó hiển thị trong combobox)
            danh_sach_ban_do.append(dict_ban_do_moi["ten_ban_do_moi"])
            return jsonify({"status": "success", "message": "Đã đặt cờ lưu bản đồ mới"})
        return jsonify({"status": "error", "message": "Tên bản đồ mới trống"})

    elif action == 'manual_control':
        dict_ban_do_moi["dieu_khien_thu_cong"] = status
        return jsonify({"status": "success", "message": "Đã cập nhật cờ điều khiển thủ công"})
    
    # Xử lý các nút mũi tên
    elif action in ["forward", "backward", "turn_left", "turn_right"]:
        if action == "forward":
            dict_ban_do_moi["tien"] = status
        elif action == "backward":
            dict_ban_do_moi["lui"] = status
        elif action == "turn_left":
            dict_ban_do_moi["trai"] = status
        elif action == "turn_right":
            dict_ban_do_moi["phai"] = status
        return jsonify({"status": "success", "message": f"Đã cập nhật trạng thái phím '{action}'"})

    return jsonify({"status": "error", "message": "Hành động không xác định"}), 400
# def update_img():
#     global danh_sach_diem, danh_sach_duong, current_image, current_image0, paint_dict_data_grid
#     # --- Draw points and paths on current_image ---
#     print("jjjjjjjjjjjjj")
#     with image_lock:
#         img_to_draw_on = current_image0.copy() # Start with the base map

#         # --- NEW: Draw grid data if enabled ---
#         if paint_dict_data_grid:
#             overlay = img_to_draw_on.copy()
#             alpha = 0.3 # Transparency factor for the rectangles

#             for grid_name, grid_data in dict_data_grid.items():
#                 vi_tri = grid_data.get("vi_tri")
#                 mau_str = grid_data.get("mau", "gray") # Default to gray if color not specified
#                 name = grid_data.get("name")
#                 diem = grid_data.get("diem") # Get diem

#                 if vi_tri and len(vi_tri) == 4:
#                     x1, y1, x2, y2 = vi_tri
#                     number = 2
#                     color_bgr = convert_color_name_to_bgr(mau_str)
                    
#                     # Draw a filled rectangle on the overlay
#                     cv2.rectangle(overlay, (x1+number, y1+number), (x2-number, y2-number), color_bgr, -1)
#                     print("ggg")
                    
#                     # Draw the grid name
#                     x_text_pos = int(x1 + (x2 - x1) * 0.1) # Adjust text position
#                     y_text_pos = int(y1 + (y2 - y1) * 0.2) # Adjust text position
#                     cv2.putText(img_to_draw_on, name, (x_text_pos, y_text_pos),
#                                         cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 255), 1, cv2.LINE_AA) # Magenta text

#                     # Draw the 'diem' point if it exists
#                     if diem and len(diem) == 2:
#                         diemX, diemY = diem
#                         # Draw a small circle for the 'diem'
#                         cv2.circle(img_to_draw_on, (int(diemX), int(diemY)), 2, (0, 0, 0), -1) # Black circle, radius 5
#                         cv2.putText(img_to_draw_on, name, (int(diemX) + 8, int(diemY) + 5),
#                                     cv2.FONT_HERSHEY_SIMPLEX, 0.2, (0, 0, 0), 1, cv2.LINE_AA) # Black text for diem coords
            
#             # Blend the overlay with the main image
#             img_to_draw_on = cv2.addWeighted(overlay, alpha, img_to_draw_on, 1 - alpha, 0)
#         # --- END NEW: Draw grid data ---

#         # Draw all points from danh_sach_diem
#         for name, p_data in danh_sach_diem.items():
#             px, py, p_type, p_angle = p_data
#             # Draw point (e.g., a red circle)
#             cv2.circle(img_to_draw_on, (int(px), int(py)), 8, (255, 0, 0), -1) # BGR: Red, radius 8
#             # Draw point name
#             cv2.putText(img_to_draw_on, name, (int(px) + 10, int(py) + 5),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 1, cv2.LINE_AA) # Magenta text

#             # Nếu điểm là "có hướng", vẽ thêm mũi tên
#             if p_type == "có hướng":
#                 arrow_length = 30  # Chiều dài mũi tên (pixel)
#                 # Giả định p_angle là độ, 0 độ hướng sang phải, tăng dần theo chiều kim đồng hồ
#                 angle_rad = math.radians(p_angle)
#                 end_x = int(px + arrow_length * math.cos(angle_rad))
#                 end_y = int(py + arrow_length * math.sin(angle_rad))
#                 cv2.arrowedLine(img_to_draw_on, (int(px), int(py)), (end_x, end_y), (0, 255, 0), 2, tipLength=0.3) # Mũi tên màu xanh lá

#         # Draw all paths from danh_sach_duong
#         for path_id, (p1_name, p2_name) in danh_sach_duong.items():
#             if p1_name in danh_sach_diem and p2_name in danh_sach_diem:
#                 p1_coords = (int(danh_sach_diem[p1_name][0]), int(danh_sach_diem[p1_name][1]))
#                 p2_coords = (int(danh_sach_diem[p2_name][0]), int(danh_sach_diem[p2_name][1]))
#                 cv2.line(img_to_draw_on, p1_coords, p2_coords, (0, 255, 255), 2) # BGR: Yellow line, thickness 2
#                 # Calculate midpoint for path name
#                 mid_x = (p1_coords[0] + p2_coords[0]) // 2
#                 mid_y = (p1_coords[1] + p2_coords[1]) // 2
#                 cv2.putText(img_to_draw_on, path_id, (mid_x, mid_y - 10), # Position text slightly above midpoint
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA) # Cyan text

#         current_image = img_to_draw_on # Update the global image to be served
#         print(f"Kích thước ảnh gửi lên web (cao, rộng, kênh): {current_image.shape}")
# Chạy ứng dụng
if __name__ == '__main__':
    app.run(debug=True)