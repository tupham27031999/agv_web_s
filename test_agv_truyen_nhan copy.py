from flask import Flask, jsonify
import requests
from requests_kerberos import HTTPKerberosAuth, REQUIRED
import os
import json
import socket
import time
from datetime import datetime

app = Flask(__name__)

# Dữ liệu AGV được tổ chức dưới dạng dictionary, với key là tên AGV.
# Sửa lại dictionary để không có key trùng lặp và có đủ 7 AGV.
dict_data_sent_dktt = {
    "agv1": {"vi_tri_hien_tai": [1,1], "dich_den": [1,3], "trang_thai": "run", "message": "None"},
    "agv2": {"vi_tri_hien_tai": [2,1], "dich_den": [2,3], "trang_thai": "run", "message": "None"},
    "agv3": {"vi_tri_hien_tai": [3,1], "dich_den": [3,3], "trang_thai": "run", "message": "None"},
    "agv4": {"vi_tri_hien_tai": [4,1], "dich_den": [4,3], "trang_thai": "run", "message": "None"},
    "agv5": {"vi_tri_hien_tai": [5,1], "dich_den": [5,3], "trang_thai": "run", "message": "None"},
    "agv6": {"vi_tri_hien_tai": [6,1], "dich_den": [6,3], "trang_thai": "run", "message": "None"},
    "agv7": {"vi_tri_hien_tai": [7,1], "dich_den": [7,3], "trang_thai": "run", "message": "None"}
}

# 1. Tạo một địa chỉ (API endpoint) để cung cấp dữ liệu AGV
@app.route('/api/agv_data', methods=['GET'])
def get_agv_data():
    """
    Endpoint này trả về dữ liệu của tất cả AGV dưới dạng JSON.
    Bất kỳ ai cũng có thể truy cập qua URL: http://<your_ip>:5000/api/agv_data
    """
    return jsonify(dict_data_sent_dktt), 200

# 2. Hàm để lấy thông tin từ một địa chỉ IP khác
def nhan_va_truyen_du_lieu_dktt(ip_address, port):
    """
    Gửi trạng thái của tất cả AGV đến server và thu thập phản hồi.
    """
    responses = {}
    base_url = f"http://{ip_address}:{port}/API/GetData"

    # Lặp qua các AGV trong dictionary dữ liệu
    for agv_name, agv_data in dict_data_sent_dktt.items():
        # Lấy dữ liệu từ dictionary một cách an toàn
        vi_tri = agv_data.get("vi_tri_hien_tai", [0,0])
        dich = agv_data.get("dich_den", [0,0])
        trang_thai = agv_data.get("trang_thai", "unknown")
        message = agv_data.get("message", "None")

        # Chuyển đổi list thành string để đưa vào URL
        vi_tri_str = ','.join(map(str, vi_tri))
        dich_str = ','.join(map(str, dich))

        # Sử dụng `params` để requests tự động mã hóa URL
        params = {
            'name_agv': agv_name,
            'vi_tri_hien_tai': vi_tri_str,
            'dich_den': dich_str,
            'trang_thai': trang_thai,
            'message': message
        }
        
        print(f"Đang gửi dữ liệu cho {agv_name}...")
        try:
            kerberos_auth = HTTPKerberosAuth(mutual_authentication=REQUIRED)
            response = requests.get(base_url, params=params, auth=kerberos_auth, timeout=10)
            
            print(f"Phản hồi từ server cho {agv_name}: {response.status_code}")

            if response.status_code == 200:
                raw_data = response.text
                try:
                    parsed_data = json.loads(raw_data)
                    responses[agv_name] = parsed_data
                except json.JSONDecodeError:
                    print(f"Lỗi: Không thể phân tích JSON từ: {raw_data}")
                    responses[agv_name] = raw_data
            else:
                print(f"Lỗi: Nhận được status code {response.status_code} cho {agv_name}")
                if response.status_code == 401:
                    print("Headers từ server:", response.headers)
                responses[agv_name] = {'error': response.status_code, 'content': response.text}

        except requests.exceptions.RequestException as e:
            print(f"Lỗi kết nối cho {agv_name}: {e}")
            responses[agv_name] = {'error': 'ConnectionError', 'message': str(e)}
    
    return responses

# Hàm kiểm tra server có sẵn sàng không
def is_server_available(ip, port, timeout=1):
    """
    Kiểm tra xem server có sẵn sàng trên IP và port cụ thể hay không bằng cách thử kết nối TCP.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((ip, port))
        s.close()
        print(f"Server {ip}:{port} is available.")
        return True
    except (socket.timeout, ConnectionRefusedError):
        print(f"Server {ip}:{port} is not available.")
    except (socket.timeout, ConnectionRefusedError, OSError) as e:
        print(f"Server {ip}:{port} is not available. Error: {e}")
        return False

# 2. Hàm để lấy thông tin từ một địa chỉ IP khác
def gui_du_lieu_cho_agv(ip_address, port, data):
# Hàm để gửi dữ liệu đến một AGV cụ thể
def gui_du_lieu_cho_agv(ip_address, port, data_to_send):
    """
    Gửi dữ liệu đến một AGV cụ thể và thu thập phản hồi.
    Gửi dữ liệu JSON đến một AGV cụ thể và thu thập phản hồi.
    """
    # Kiểm tra xem server có đang bật không trước khi gửi
    if not is_server_available(ip_address, port):
        error_message = f"Không thể kết nối đến server {ip_address}:{port}. Bỏ qua việc gửi dữ liệu."
        print(error_message)
        return {'error': 'ServerUnavailable', 'message': error_message}

    responses = {}
    base_url = f"http://{ip_address}:{port}/PC_sent_AGV_1"
    # Endpoint trên server chính (webserver.py)
    # Lưu ý: Endpoint trong webserver.py là '/PC_sent_AGV', không phải '/PC_sent_AGV_1'
    endpoint = "PC_sent_AGV"
    base_url = f"http://{ip_address}:{port}/{endpoint}"
    print(f"Gửi dữ liệu đến {base_url}...")

    # Server mong đợi một JSON object có key là 'signal'.
    # Giá trị của 'signal' sẽ là dictionary dữ liệu của AGV.
    payload = {'signal': data}
    payload = {'signal': data_to_send}
    print("Payload to send:", json.dumps(payload, indent=4, ensure_ascii=False))

    try:
        # Sử dụng POST và gửi payload dưới dạng JSON.
        # Đã loại bỏ xác thực Kerberos không cần thiết.
        response = requests.post(base_url, json=payload, timeout=10) # timeout sau 10 giây
        response = requests.post(base_url, json=payload, timeout=10)
        
        # Kiểm tra xem yêu cầu có thành công không (status code 2xx)
        response.raise_for_status()

        print(f"Phản hồi từ server: {response.status_code}")
        
        if response.status_code == 200:
            try:
                responses = response.json()
                print("Phản hồi JSON từ server:", responses)
            except json.JSONDecodeError:
                print("Không thể phân tích JSON từ server. Nội dung:", response.text)
                responses = {'error': 'JSONDecodeError', 'content': response.text}
        else:
            print(f"Lỗi: Nhận được status code {response.status_code}")
            responses = {'error': response.status_code, 'content': response.text}

        try:
            responses = response.json()
            print("Phản hồi JSON từ server:", responses)
            return responses
        except json.JSONDecodeError:
            print("Không thể phân tích JSON từ server. Nội dung:", response.text)
            return {'error': 'JSONDecodeError', 'content': response.text}

    except requests.exceptions.HTTPError as errh:
        print(f"Lỗi HTTP: {errh}")
        print(f"Nội dung phản hồi: {errh.response.text}")
        return {'error': 'HTTPError', 'status_code': errh.response.status_code, 'content': errh.response.text}
    except requests.exceptions.RequestException as e:
        print(f"Lỗi kết nối: {e}")
        responses = {'error': 'RequestException', 'message': str(e)}
    
    return responses
        return {'error': 'RequestException', 'message': str(e)}

def update_agv_data(dict_agv, received_data):
    """
    Tìm và cập nhật dữ liệu cho một AGV trong dictionary dựa trên dữ liệu nhận được.
    """
    agv_name_to_update = received_data.get("name_agv")
    if not agv_name_to_update:
        print("Dữ liệu nhận được không có 'name_agv'")
        return False

    if agv_name_to_update in dict_agv:
        print(f"Đang cập nhật dữ liệu cho {agv_name_to_update}...")
        # Cập nhật các khóa có trong dữ liệu nhận được
        dict_agv[agv_name_to_update].update(received_data)
        return True
# Chạy kịch bản client
if __name__ == '__main__':
    # Dữ liệu mẫu để gửi đi
    data_agv1 = {'name_agv': 'agv1', 'dich_den': [4, 3], 'trang_thai': 'run'}
    
    print(f"Không tìm thấy AGV có tên '{agv_name_to_update}' trong danh sách.")
    return False

def log_communication(data_to_log, log_type='nhan'):
    """
    Ghi lại thông tin giao tiếp vào file log theo ngày và giờ.
    """
    LOG_DIR = "data_input_output/log_giao_tiep"
    os.makedirs(LOG_DIR, exist_ok=True)
    # Địa chỉ IP và Port của server chính (chạy file main.py)
    # Bạn cần thay đổi IP này thành IP của máy chạy server chính
    SERVER_IP = "192.168.143.1" 
    SERVER_PORT = 5000
    
    now = datetime.now()
    log_filename = f"log_{log_type}_{now.strftime('%Y-%m-%d_%H')}.txt"
    log_filepath = os.path.join(LOG_DIR, log_filename)
    print("--- Bắt đầu kịch bản test gửi dữ liệu ---")
    
    log_entry = f"{now.strftime('%Y-%m-%d %H:%M:%S')}\t{data_to_log}\n"
    
    try:
        with open(log_filepath, 'a', encoding='utf-8') as f:
            f.write(log_entry)
        print(f"Đã ghi log vào: {log_filepath}")
    except Exception as e:
        print(f"Lỗi khi ghi log: {e}")
# {'agv1': {'name_agv': 'agv1', 'dich_den': [4, 3], 'trang_thai': 'run'}, 
#  'agv2': {'name_agv': 'agv2', 'dich_den': [12, 3], 'trang_thai': 'run'}, 
#  'agv3': {'name_agv': 'agv3', 'dich_den': [4, 5], 'trang_thai': 'run'}, 
#  'agv4': {'name_agv': 'agv4', 'dich_den': [12, 5], 'trang_thai': 'run'}, 
#  'agv5': {'name_agv': 'agv5', 'dich_den': [4, 7], 'trang_thai': 'run'}, 
#  'agv6': {'name_agv': 'agv6', 'dich_den': [12, 7], 'trang_thai': 'run'}, 
#  'agv7': {'name_agv': 'agv7', 'dich_den': [4, 9], 'trang_thai': 'run'}}
# Chạy web server
if __name__ == '__main__':
    # all_responses = nhan_va_truyen_du_lieu_dktt("apbivnwb06", 1332)
    all_responses = {'agv1': {'name_agv': 'agv1', 'dich_den': [4, 3], 'trang_thai': 'run'}, 
                    'agv2': {'name_agv': 'agv2', 'dich_den': [12, 3], 'trang_thai': 'run'}, 
                    'agv3': {'name_agv': 'agv3', 'dich_den': [4, 5], 'trang_thai': 'run'}, 
                    'agv4': {'name_agv': 'agv4', 'dich_den': [12, 5], 'trang_thai': 'run'}, 
                    'agv5': {'name_agv': 'agv5', 'dich_den': [4, 7], 'trang_thai': 'run'}, 
                    'agv6': {'name_agv': 'agv6', 'dich_den': [12, 7], 'trang_thai': 'run'}, 
                    'agv7': {'name_agv': 'agv7', 'dich_den': [4, 9], 'trang_thai': 'run'}}
    
    # http://10.116.3.1:5000/
    gui_du_lieu_cho_agv("10.116.3.1", 5000, all_responses['agv1'])
    # Chờ một chút để đảm bảo server chính đã khởi động (nếu cần)
    print("Đang chờ 2 giây để server có thể khởi động...")
    time.sleep(2)

    # if all_responses:
    #     print("\n--- Tổng hợp các phản hồi từ server ---")
    #     print(all_responses)
    #     print(json.dumps(all_responses, indent=4, ensure_ascii=False))
    # Gửi dữ liệu của AGV1
    response_from_server = gui_du_lieu_cho_agv(SERVER_IP, SERVER_PORT, data_agv1)

    print("\n--- Kết quả từ server ---")
    if response_from_server:
        print(json.dumps(response_from_server, indent=4, ensure_ascii=False))
    else:
        print("Không nhận được phản hồi hợp lệ từ server.")
        
        # # Ghi log và cập nhật dữ liệu cho từng phản hồi
        # for agv_name, response_data in all_responses.items():
        #     log_communication(f"{agv_name}: {json.dumps(response_data, ensure_ascii=False)}", log_type='nhan')
        #     if isinstance(response_data, dict) and 'error' not in response_data:
        #         update_agv_data(dict_data_sent_dktt, response_data)
        
        # print("\n--- Danh sách AGV sau khi cập nhật ---")
        # print(json.dumps(dict_data_sent_dktt, indent=4, ensure_ascii=False))
    # else:
    #     print("không có dữ liệu")
    print("\n--- Kịch bản test đã hoàn tất. File này không khởi động server và sẽ tự kết thúc. ---")
    
    print("\n--- Khởi động server Flask ---")
    app.run(host='0.0.0.0', port=9000, debug=False)