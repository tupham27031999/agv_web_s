# Cài đặt thư viện requests nếu bạn chưa có:
# pip install requests

import requests

# ----------------------------------------------------
# 1. Ví dụ gửi yêu cầu GET
# ----------------------------------------------------

def send_get_request():
    """Gửi một yêu cầu GET tới một URL và in kết quả."""
    # Định nghĩa URL của máy chủ cần truy cập
    # Dùng địa chỉ IP của server Flask bạn đã chạy
    # url_get = 'http://10.116.3.1:50000'
    url_get = "http://10.116.3.1:50000"

    print(f"Đang gửi yêu cầu GET tới: {url_get}")
    try:
        # Gửi yêu cầu GET
        response = requests.get(url_get)

        # Kiểm tra mã trạng thái của phản hồi (status code)
        if response.status_code == 200:
            # Lấy dữ liệu trả về và in ra
            data = response.text
            print("Phản hồi nhận được:")
            print(data)
            print("-" * 50)
        else:
            print(f"Yêu cầu không thành công. Mã trạng thái: {response.status_code}")
            print("-" * 50)
    except requests.exceptions.RequestException as e:
        print(f"Có lỗi xảy ra khi kết nối: {e}")
        print("-" * 50)


# ----------------------------------------------------
# 2. Ví dụ gửi yêu cầu POST
# ----------------------------------------------------
url_unpause = "http://10.116.3.2:50000/v2/missions-management/skip-action"
url_resume = "http://10.116.3.2:50000/v2/missions-management/resume"
def send_post_request():
    """Gửi một yêu cầu POST với dữ liệu JSON tới một URL và in kết quả."""
    # Định nghĩa URL của máy chủ
    url_post = url_unpause

    # Dữ liệu cần gửi đi dưới dạng từ điển Python
    post_data = {
        'name': 'Johnmmmm'
    }

    print(f"Đang gửi yêu cầu POST tới: {url_post}")
    print(f"Dữ liệu gửi đi: {post_data}")
    try:
        # Gửi yêu cầu POST với dữ liệu
        response = requests.post(url_post, json=post_data)

        # Kiểm tra mã trạng thái
        if response.status_code == 201:
            # Lấy dữ liệu phản hồi
            response_data = response.json()
            print("Phản hồi nhận được:")
            print(response_data)
        else:
            print(f"Yêu cầu không thành công. Mã trạng thái: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Có lỗi xảy ra khi kết nối: {e}")


# Gọi các hàm để chạy thử
if __name__ == '__main__':
    send_get_request()
    send_post_request()
