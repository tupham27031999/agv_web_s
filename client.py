import time
import requests

# Địa chỉ IP và port của server
SERVER_IP = '10.116.3.1'
SERVER_PORT = 50000

# Các URL để giao tiếp
url_get = f"http://{SERVER_IP}:{SERVER_PORT}"
url_unpause = f"http://{SERVER_IP}:{SERVER_PORT}/v2/missions-management/skip-action"
url_resume = f"http://{SERVER_IP}:{SERVER_PORT}/v2/missions-management/resume"

def run_client_sequence():
    try:
        # Bước 1: Gửi yêu cầu GET
        print(f"Bước 1: Gửi yêu cầu GET đến {url_get}")
        response_get = requests.get(url_get, timeout=10)
        print(f"Nhận được phản hồi từ server: {response_get.json()}")
        
        # Kiểm tra phản hồi
        if response_get.json().get('status') == 'ok1':
            print("Phản hồi thành công, chờ 2 giây trước khi gửi POST...")
            time.sleep(2)

            # Bước 2: Gửi yêu cầu POST đến skip-action
            print(f"Bước 2: Gửi yêu cầu POST đến {url_unpause}")
            response_unpause = requests.post(url_unpause, timeout=10)
            print(f"Server trả lời: {response_unpause.status_code}")
            
            # Chờ 1 giây
            print("Đang chờ 1 giây trước khi gửi yêu cầu POST tiếp theo...")
            time.sleep(1)

            # Bước 3: Gửi yêu cầu POST đến resume
            print(f"Bước 3: Gửi yêu cầu POST đến {url_resume}")
            response_resume = requests.post(url_resume, timeout=10)
            print(f"Server trả lời: {response_resume.status_code}")

            print("\n✔️ Hoàn tất chuỗi giao tiếp.")

    except requests.exceptions.RequestException as e:
        print(f"Lỗi kết nối: {e}")

if __name__ == '__main__':
    # Chạy client
    run_client_sequence()