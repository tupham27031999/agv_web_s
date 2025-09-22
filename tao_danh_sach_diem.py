import json

def generate_grid_points(start_x, start_y, dx, dy, cols, rows):
    points = {}
    idx = 1
    for r in range(rows):
        for c in range(cols):
            x = start_x + c * dx
            y = start_y + r * dy
            points[f"P{idx}"] = [x, y, "không hướng", 0.0]
            idx += 1
    return points

# Ví dụ sử dụng
if __name__ == "__main__":
    start_x = 1042
    start_y = 924
    dx = 50   # khoảng cách cột
    dy = 50   # khoảng cách hàng
    cols = 5  # số cột
    rows = 3  # số hàng

    grid = generate_grid_points(start_x, start_y, dx, dy, cols, rows)

    # Lưu ra file JSON giống format ban đầu
    with open("grid_points.json", "w", encoding="utf-8") as f:
        json.dump(grid, f, ensure_ascii=False, indent=4)

    print("Tạo xong grid_points.json")
