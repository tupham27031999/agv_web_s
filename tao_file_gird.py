import json
import os

def generate_grid_data(x_start, y_start, width, height, num_cols, num_rows, file_name="output.txt"):
    """
    Tạo dữ liệu lưới các điểm và lưu vào file TXT (JSON format).

    Args:
        x_start (int): Tọa độ X bắt đầu của điểm đầu tiên.
        y_start (int): Tọa độ Y bắt đầu của điểm đầu tiên.
        width (int): Chiều rộng của mỗi ô lưới.
        height (int): Chiều cao của mỗi ô lưới.
        num_cols (int): Số lượng cột (điểm theo chiều ngang).
        num_rows (int): Số lượng hàng (điểm theo chiều dọc).
        file_name (str): Tên file để lưu dữ liệu. Mặc định là "output.txt".
    """
    if os.path.exists(file_name) == True:
        os.remove(file_name)
    data = {}
    
    for r in range(num_rows):
        for c in range(num_cols):
            # Tính toán tọa độ cho từng ô lưới
            # diem: tọa độ trung tâm của ô
            diem_x = x_start + c * width + width // 2
            diem_y = y_start + r * height + height // 2
            
            # vi_tri: bounding box [x_min, y_min, x_max, y_max]
            vi_tri_x1 = x_start + c * width
            vi_tri_y1 = y_start + r * height
            vi_tri_x2 = x_start + (c + 1) * width
            vi_tri_y2 = y_start + (r + 1) * height
            
            # Tạo tên cho điểm theo định dạng "cot.hang" (ví dụ: "0.0", "1.0", "0.1")
            name = f"{c}.{r}"
            
            # Nếu bạn muốn tên có định dạng số thứ tự tăng dần (0, 1, 2...)
            # index = r * num_cols + c
            # name = str(index)

            # Tạo cấu trúc dữ liệu cho từng điểm
            point_data = {
                "diem": [diem_x, diem_y],
                "loai_diem": "duong_di",
                "mau": "blue",
                "name": name,
                "vi_tri": [vi_tri_x1, vi_tri_y1, vi_tri_x2, vi_tri_y2]
            }
            data[name] = point_data

    # Ghi dữ liệu vào file JSON
    try:
        with open(file_name, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=4, ensure_ascii=False)
        print(f"Dữ liệu đã được tạo và lưu vào file '{file_name}' thành công.")
    except IOError as e:
        print(f"Lỗi khi ghi file '{file_name}': {e}")


# Gọi hàm để tạo dữ liệu
# x_start, y_start, width, height, num_cols, num_rows, file_name
generate_grid_data(800, 800, 30, 30, 5, 5, "grid_data.json")