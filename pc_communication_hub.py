from flask import Flask, request, jsonify, render_template_string
import threading
import time
import requests
import os
import json
from datetime import datetime
from support_main.lib_main import edit_csv_tab
import path

path_phan_mem = path.path_phan_mem
path_agv_pc_config = path_phan_mem + "/setting/agv_pc_config.csv"

# gửi dữ liệu dựa vào id của agv ví dụ AGV_CONFIGS["agv_id"]
AGV_CONFIGS = {} # Stores config for each AGV: {agv_id: {"ui_label": ui_label, "send_url": send_url, "get_url": get_url}} pc_hub_logs
PC_CONFIG = {} # {"ui_host": "", "ui_port": "", "upstream_host": "", "upstream_port": "", "pc_log_dir": "", "pc_hub_logs": ""}

data_agv_pc_config = edit_csv_tab.load_all_stt(path_agv_pc_config)
default_event_log_dir = path_phan_mem + "/data_input_output/pc_hub_event_logs_default"
default_signal_log_dir = path_phan_mem + "/data_input_output/pc_hub_signal_logs_default"
for i in range(0,len(data_agv_pc_config)):
    if len(data_agv_pc_config[i]) > 1:
        if data_agv_pc_config[i][0] == "pc_hub_ui_host":
            PC_CONFIG["upstream_host"] = data_agv_pc_config[i][1]
        if data_agv_pc_config[i][0] == "pc_hub_ui_port":
            PC_CONFIG["upstream_port"] = int(float(data_agv_pc_config[i][1]))

        if data_agv_pc_config[i][0] == "num_agvs":
            num_agvs = int(float(data_agv_pc_config[i][1]))

        if data_agv_pc_config[i][0] == "pc_hub_ui_host":
            PC_CONFIG["ui_host"] = data_agv_pc_config[i][1]
        if data_agv_pc_config[i][0] == "pc_hub_ui_port":
            PC_CONFIG["ui_port"] = int(float(data_agv_pc_config[i][1]))

        if data_agv_pc_config[i][0] == "AGV":
            AGV_CONFIGS[data_agv_pc_config[i][1]] = {"ui_label": data_agv_pc_config[i][2], "send_url": data_agv_pc_config[i][3], "get_url": data_agv_pc_config[i][4]}

        if data_agv_pc_config[i][0] == "log":
            PC_CONFIG["pc_event_log_dir"] = data_agv_pc_config[i][1] if data_agv_pc_config[i][1] else default_event_log_dir
            PC_CONFIG["pc_signal_log_dir"] = data_agv_pc_config[i][2] if data_agv_pc_config[i][2] else default_signal_log_dir

        


app = Flask(__name__)

# --- Configuration ---
CONFIG_FILE_PATH = os.path.join('setting', 'agv_pc_config.txt')


# --- Data Storage for PC Hub ---
upstream_signal_data = {"signal": "N/A", "timestamp": "N/A", "status": "Chưa nhận tín hiệu upstream."}

# Stores the latest signal PC *intends* to send or *has sent* to each AGV
# { agv_id: {"signal_to_send": "N/A", "last_sent_timestamp": "N/A", "last_send_status": "Chưa gửi"} }
agv_signals_to_pc = {}

# Stores the latest signal PC *has received* from each AGV
# { agv_id: {"signal_received": "N/A", "last_received_timestamp": "N/A", "last_receive_status": "Chưa nhận"} }
agv_signals_from_pc = {}

log_lock = threading.Lock()
PC_EVENT_LOG_DIR = default_event_log_dir

# Thêm biến cho thư mục log giao tiếp và lock
PC_SIGNAL_LOG_DIR = default_signal_log_dir
comm_log_lock = threading.Lock()

def setup_logging():
    global PC_EVENT_LOG_DIR, PC_SIGNAL_LOG_DIR
    
    configured_event_log_dir = PC_CONFIG.get("pc_event_log_dir")
    if configured_event_log_dir:
        PC_EVENT_LOG_DIR = configured_event_log_dir
    os.makedirs(PC_EVENT_LOG_DIR, exist_ok=True)
    print(f"Thư mục log sự kiện PC Hub được đặt thành: {os.path.abspath(PC_EVENT_LOG_DIR)}")

    configured_signal_log_dir = PC_CONFIG.get("pc_signal_log_dir")
    if configured_signal_log_dir:
        PC_SIGNAL_LOG_DIR = configured_signal_log_dir
    os.makedirs(PC_SIGNAL_LOG_DIR, exist_ok=True)
    print(f"Thư mục log giao tiếp tín hiệu được đặt thành: {os.path.abspath(PC_SIGNAL_LOG_DIR)}")

def log_pc_event(event_type, message):
    try:
        with log_lock:
            now = datetime.now()
            date_hour_str = now.strftime("%Y-%m-%d_%H")
            timestamp_str = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            filename = f"pc_hub_log_{date_hour_str}.txt"
            filepath = os.path.join(PC_EVENT_LOG_DIR, filename)

            with open(filepath, "a", encoding="utf-8") as f:
                f.write(f"{timestamp_str}\t[{event_type.upper()}]\t{message}\n")
    except Exception as e:
        print(f"Error writing to PC log file: {e}")

def log_signal_communication(log_direction_tag, target_id, signal_content, event_timestamp_str, status_message=""):
    """
    Ghi log giao tiếp tín hiệu.
    log_direction_tag: "PC_TO_AGV", "AGV_TO_PC", "UPSTREAM_TO_PC"
    target_id: AGV ID hoặc "UPSTREAM"
    signal_content: Nội dung tín hiệu
    event_timestamp_str: Thời gian của sự kiện (đã được format YYYY-MM-DD HH:MM:SS.mmm)
    status_message: Thông điệp trạng thái thêm (ví dụ: lỗi, thành công)
    """
    try:
        with comm_log_lock:
            # Parse timestamp to get hour for filename (ignore milliseconds for parsing if present)
            dt_object = datetime.strptime(event_timestamp_str.split('.')[0], "%Y-%m-%d %H:%M:%S")
            date_hour_str = dt_object.strftime("%Y-%m-%d_%H")

            safe_target_id = "".join(c if c.isalnum() else "_" for c in str(target_id))
            filename = f"{log_direction_tag}_{safe_target_id}_{date_hour_str}.txt"
            filepath = os.path.join(PC_SIGNAL_LOG_DIR, filename)

            with open(filepath, "a", encoding="utf-8") as f:
                log_entry = f"{event_timestamp_str}\t{signal_content}"
                if status_message:
                    log_entry += f"\t{status_message}"
                f.write(log_entry + "\n")
    except Exception as e:
        print(f"Lỗi ghi vào file log tín hiệu: {e}")
        log_pc_event("error", f"Không thể ghi vào file log tín hiệu {filepath}: {e}")

# --- Communication with AGVs ---
def send_signal_to_agv(agv_id, signal_payload_str):
    if agv_id not in AGV_CONFIGS:
        log_pc_event("error", f"send_signal_to_agv: AGV ID '{agv_id}' không tồn tại trong cấu hình.")
        return False, "AGV ID không hợp lệ"

    url = AGV_CONFIGS[agv_id]["send_url"]
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    try:
        response = requests.post(url, json={"signal": signal_payload_str}, timeout=5)
        response.raise_for_status() # Will raise an HTTPError for bad responses (4XX or 5XX)
        response_data = response.json()

        agv_signals_to_pc[agv_id]["signal_to_send"] = signal_payload_str
        agv_signals_to_pc[agv_id]["last_sent_timestamp"] = timestamp
        status_msg = f"SUCCESS: {response.status_code} - {response_data.get('message', 'OK')}"
        agv_signals_to_pc[agv_id]["last_send_status"] = f"Đã gửi: {status_msg}"
        log_pc_event("info", f"Gửi đến {agv_id} ({url}): '{signal_payload_str}'. Phản hồi: {response.status_code} {response.text}")
        log_signal_communication("PC_TO_AGV", agv_id, signal_payload_str, timestamp, status_msg)
        return True, agv_signals_to_pc[agv_id]["last_send_status"]
    except requests.exceptions.RequestException as e:
        error_msg = f"FAIL_CONNECTION: {str(e)}"
        agv_signals_to_pc[agv_id]["last_send_status"] = f"Lỗi gửi: {error_msg}"
        agv_signals_to_pc[agv_id]["last_sent_timestamp"] = timestamp
        log_pc_event("error", f"Lỗi gửi đến {agv_id} ({url}): '{signal_payload_str}'. Lỗi: {str(e)}")
        log_signal_communication("PC_TO_AGV", agv_id, signal_payload_str, timestamp, error_msg)
        return False, str(e)

def poll_agv_signals():
    while True:
        for agv_id, config in AGV_CONFIGS.items():
            url = config["get_url"]
            pc_timestamp_at_poll = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            try:
                response = requests.get(url, timeout=3)
                response.raise_for_status()
                data = response.json()
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                if data.get("status") == "success":
                    agv_signals_from_pc[agv_id]["signal_received"] = data.get("signal", "N/A")
                    agv_signals_from_pc[agv_id]["last_received_timestamp"] = data.get("timestamp", timestamp) # Use AGV's timestamp if available
                    status_msg = "SUCCESS"
                    agv_signals_from_pc[agv_id]["last_receive_status"] = f"Đã nhận thành công lúc (AGV time): {agv_signals_from_pc[agv_id]['last_received_timestamp']}"
                    log_pc_event("info", f"Nhận từ {agv_id} ({url}): '{agv_signals_from_pc[agv_id]['signal_received']}' lúc (AGV time) {agv_signals_from_pc[agv_id]['last_received_timestamp']}")
                    log_signal_communication("AGV_TO_PC", agv_id, agv_signals_from_pc[agv_id]["signal_received"], agv_signals_from_pc[agv_id]["last_received_timestamp"], status_msg)
                else:
                    status_msg = f"FAIL_AGV_STATUS: {data.get('message', 'Trạng thái không thành công từ AGV')}"
                    agv_signals_from_pc[agv_id]["last_receive_status"] = f"Lỗi nhận: {status_msg}"
                    agv_signals_from_pc[agv_id]["last_received_timestamp"] = pc_timestamp_at_poll
                    log_pc_event("warn", f"Nhận từ {agv_id} ({url}) không thành công: {status_msg}")
                    log_signal_communication("AGV_TO_PC", agv_id, "N/A_AGV_ERROR", pc_timestamp_at_poll, status_msg)

            except requests.exceptions.RequestException as e:
                status_msg = f"FAIL_CONNECTION: {str(e)}"
                agv_signals_from_pc[agv_id]["last_receive_status"] = f"Lỗi kết nối: {status_msg}"
                agv_signals_from_pc[agv_id]["last_received_timestamp"] = pc_timestamp_at_poll
                log_pc_event("error", f"Lỗi kết nối khi nhận từ {agv_id} ({url}): {str(e)}")
                log_signal_communication("AGV_TO_PC", agv_id, "N/A_CONNECTION_ERROR", pc_timestamp_at_poll, status_msg)
            except json.JSONDecodeError:
                status_msg = "FAIL_JSON_DECODE: Phản hồi không phải JSON"
                agv_signals_from_pc[agv_id]["last_receive_status"] = f"Lỗi: {status_msg}"
                agv_signals_from_pc[agv_id]["last_received_timestamp"] = pc_timestamp_at_poll
                log_pc_event("error", f"Phản hồi từ {agv_id} ({url}) không phải JSON hợp lệ.")
                log_signal_communication("AGV_TO_PC", agv_id, "N/A_INVALID_JSON", pc_timestamp_at_poll, status_msg)
        time.sleep(5) # Poll interval

# --- Flask Routes ---
@app.route('/')
def index():
    # HTML content will be generated dynamically based on AGV_CONFIGS
    agv_sections_html = ""
    for agv_id, config in AGV_CONFIGS.items():
        agv_sections_html += f"""
        <div class="agv-card">
            <h3>{config['ui_label']} (ID: {agv_id})</h3>
            <div class="comm-section">
                <h4>PC Gửi đến {config['ui_label']}</h4>
                <p>Tín hiệu gửi: <span id="sent_signal_{agv_id}">N/A</span></p>
                <p>Thời gian gửi: <span id="sent_timestamp_{agv_id}">N/A</span></p>
                <p>Trạng thái gửi: <span id="sent_status_{agv_id}">Chưa gửi</span></p>
                <input type="text" id="manual_signal_{agv_id}" placeholder="Nhập tín hiệu cho {config['ui_label']}">
                <button onclick="sendManualSignal('{agv_id}')">Gửi Thủ Công</button>
            </div>
            <div class="comm-section">
                <h4>{config['ui_label']} Gửi đến PC</h4>
                <p>Tín hiệu nhận: <span id="received_signal_{agv_id}">N/A</span></p>
                <p>Thời gian nhận: <span id="received_timestamp_{agv_id}">N/A</span></p>
                <p>Trạng thái nhận: <span id="received_status_{agv_id}">Chưa nhận</span></p>
            </div>
        </div>
        """

    return render_template_string(f"""
    <!DOCTYPE html>
    <html lang="vi">
    <head>
        <meta charset="UTF-8">
        <title>PC Communication Hub</title>
        <style>
            body {{ font-family: Arial, sans-serif; margin: 20px; background-color: #f4f4f4; color: #333; }}
            .container {{ max-width: 1200px; margin: auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }}
            h1, h2, h3 {{ color: #0056b3; }}
            .main-header {{ text-align: center; margin-bottom: 20px; padding-bottom:10px; border-bottom: 2px solid #0056b3;}}
            .section {{ background-color: #e9ecef; padding: 15px; border-radius: 5px; margin-bottom: 20px; }}
            .agv-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(350px, 1fr)); gap: 20px; }}
            .agv-card {{ border: 1px solid #ccc; border-radius: 5px; padding: 15px; background-color: #f9f9f9; }}
            .agv-card h3 {{ margin-top: 0; border-bottom: 1px solid #ddd; padding-bottom: 5px; }}
            .comm-section h4 {{ color: #007bff; margin-top: 10px; margin-bottom: 5px;}}
            p {{ margin: 5px 0; }}
            span {{ font-weight: bold; color: #28a745; }}
            input[type="text"] {{ width: calc(100% - 22px); padding: 8px; margin-top: 5px; margin-bottom: 10px; border: 1px solid #ccc; border-radius: 4px; }}
            button {{ background-color: #007bff; color: white; padding: 10px 15px; border: none; border-radius: 4px; cursor: pointer; font-size: 1em; }}
            button:hover {{ background-color: #0056b3; }}
            .status-ok {{ color: green; }}
            .status-error {{ color: red; }}
            .status-pending {{ color: orange; }}
        </style>
    </head>
    <body>
        <div class="container">
            <div class="main-header"><h1>PC AGV Communication Hub</h1></div>

            <div class="section" id="upstream-comm">
                <h2>Giao tiếp Upstream (Từ Hệ thống ngoài đến PC)</h2>
                <p>Tín hiệu nhận: <span id="upstream_signal">N/A</span></p>
                <p>Thời gian nhận: <span id="upstream_timestamp">N/A</span></p>
                <p>Trạng thái: <span id="upstream_status">Chưa nhận tín hiệu.</span></p>
            </div>

            <h2>Giao tiếp PC và AGVs</h2>
            <div class="agv-grid" id="agv-communications">
                {agv_sections_html}
            </div>
        </div>

        <script>
            function updateDashboard() {{
                fetch('/api/dashboard_data')
                    .then(response => response.json())
                    .then(data => {{
                        document.getElementById('upstream_signal').textContent = data.upstream.signal;
                        document.getElementById('upstream_timestamp').textContent = data.upstream.timestamp;
                        document.getElementById('upstream_status').textContent = data.upstream.status;

                        for (const agv_id in data.agvs_to_pc) {{
                            if (document.getElementById(`sent_signal_` + agv_id)) {{
                                document.getElementById(`sent_signal_` + agv_id).textContent = data.agvs_to_pc[agv_id].signal_to_send;
                                document.getElementById(`sent_timestamp_` + agv_id).textContent = data.agvs_to_pc[agv_id].last_sent_timestamp;
                                document.getElementById(`sent_status_` + agv_id).textContent = data.agvs_to_pc[agv_id].last_send_status;
                            }}
                        }}
                        for (const agv_id in data.agvs_from_pc) {{
                             if (document.getElementById(`received_signal_` + agv_id)) {{
                                document.getElementById(`received_signal_` + agv_id).textContent = data.agvs_from_pc[agv_id].signal_received;
                                document.getElementById(`received_timestamp_` + agv_id).textContent = data.agvs_from_pc[agv_id].last_received_timestamp;
                                document.getElementById(`received_status_` + agv_id).textContent = data.agvs_from_pc[agv_id].last_receive_status;
                            }}
                        }}
                    }})
                    .catch(error => console.error('Error fetching dashboard data:', error));
            }}

            function sendManualSignal(agvId) {{
                const signal = document.getElementById('manual_signal_' + agvId).value;
                if (!signal) {{
                    alert('Vui lòng nhập tín hiệu để gửi.');
                    return;
                }}
                fetch('/api/send_manual_to_agv', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ agv_id: agvId, signal: signal }})
                }})
                .then(response => response.json())
                .then(data => {{
                    alert(data.message);
                    if(data.status === 'success') {{
                        document.getElementById('manual_signal_' + agvId).value = ''; // Clear input
                    }}
                    updateDashboard(); // Refresh dashboard
                }})
                .catch(error => {{
                    console.error('Error sending manual signal:', error);
                    alert('Lỗi gửi tín hiệu thủ công.');
                }});
            }}

            setInterval(updateDashboard, 2000); // Update every 2 seconds
            window.onload = updateDashboard; // Initial load
        </script>
    </body>
    </html>
    """)

@app.route('/api/dashboard_data', methods=['GET'])
def get_dashboard_data():
    return jsonify({
        "upstream": upstream_signal_data,
        "agvs_to_pc": agv_signals_to_pc,
        "agvs_from_pc": agv_signals_from_pc
    })

@app.route('/api/upstream_receive', methods=['POST'])
def receive_upstream_signal():
    global upstream_signal_data
    try:
        data = request.get_json()
        if not data or 'signal' not in data:
            log_pc_event("error", f"Upstream receive: Dữ liệu không hợp lệ hoặc thiếu 'signal'. Data: {data}")
            return jsonify({"status": "error", "message": "Dữ liệu không hợp lệ hoặc thiếu 'signal'."}), 400

        signal_content = data.get('signal') # dữ liệu gửi agv
        target_agv_suggestion = data.get('target_agv_id') # id agv nhận

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        upstream_signal_data["signal"] = str(signal_content)
        upstream_signal_data["timestamp"] = timestamp
        upstream_signal_data["status"] = f"Đã nhận: '{signal_content}'"
        log_pc_event("info", f"Upstream received: '{signal_content}'. Target suggestion: {target_agv_suggestion}")
        log_signal_communication("UPSTREAM_TO_PC", "UPSTREAM", str(signal_content), timestamp, "RECEIVED")

        # --- Business Logic to process upstream_signal and send to an AGV ---
        # This is a placeholder. You need to implement how to choose an AGV
        # and what signal to send to it based on 'signal_content'.
        if target_agv_suggestion and target_agv_suggestion in AGV_CONFIGS:
            agv_to_command = target_agv_suggestion
            # Example: forward the upstream signal directly, or transform it
            signal_for_agv = f"UPSTREAM_CMD:{signal_content}" 
            success, msg = send_signal_to_agv(agv_to_command, signal_for_agv)
            if success:
                log_pc_event("dispatch", f"Đã gửi lệnh '{signal_for_agv}' đến {agv_to_command} dựa trên tín hiệu upstream.")
            else:
                log_pc_event("error", f"Lỗi gửi lệnh '{signal_for_agv}' đến {agv_to_command}. Lỗi: {msg}")
        elif AGV_CONFIGS: # If no suggestion, send to the first configured AGV as an example
            agv_to_command = list(AGV_CONFIGS.keys())[0]
            signal_for_agv = f"DEFAULT_CMD_FOR_UPSTREAM:{signal_content}"
            log_pc_event("dispatch", f"Upstream signal '{signal_content}' received. No valid target, attempting to send default command to {agv_to_command}.")
            send_signal_to_agv(agv_to_command, signal_for_agv)
        else:
            log_pc_event("warn", "Nhận tín hiệu upstream nhưng không có AGV nào được cấu hình để gửi lệnh.")
            upstream_signal_data["status"] += " (Không có AGV nào để gửi lệnh)"

        return jsonify({"status": "success", "message": "Tín hiệu upstream đã nhận và xử lý (placeholder)."}), 200

    except Exception as e:
        log_pc_event("error", f"Lỗi xử lý tín hiệu upstream: {str(e)}. Data: {request.data}")
        return jsonify({"status": "error", "message": f"Lỗi server: {str(e)}"}), 500

@app.route('/api/send_manual_to_agv', methods=['POST'])
def send_manual_signal_route():
    data = request.get_json()
    agv_id = data.get('agv_id')
    signal_str = data.get('signal')

    if not agv_id or not signal_str:
        log_pc_event("error", f"Send manual: Thiếu agv_id hoặc signal. Data: {data}")
        return jsonify({"status": "error", "message": "Thiếu agv_id hoặc signal."}), 400

    if agv_id not in AGV_CONFIGS:
        log_pc_event("error", f"Send manual: AGV ID '{agv_id}' không hợp lệ. Data: {data}")
        return jsonify({"status": "error", "message": f"AGV ID '{agv_id}' không hợp lệ."}), 400

    success, message = send_signal_to_agv(agv_id, signal_str)

    if success:
        return jsonify({"status": "success", "message": f"Đã gửi '{signal_str}' đến {agv_id}. Status: {message}"})
    else:
        return jsonify({"status": "error", "message": f"Lỗi gửi '{signal_str}' đến {agv_id}: {message}"}), 500


if __name__ == '__main__':
    # Khởi tạo thư mục log
    setup_logging()

    # Initialize AGV-specific data stores based on loaded AGV_CONFIGS.
    # This ensures that agv_signals_to_pc and agv_signals_from_pc are populated
    # with entries for all configured AGVs before the polling thread or any
    # request handlers try to access them with specific agv_ids.
    for agv_id_key in AGV_CONFIGS.keys():
        # agv_signals_to_pc is used by send_signal_to_agv and /api/dashboard_data
        agv_signals_to_pc[agv_id_key] = {
            "signal_to_send": "N/A",
            "last_sent_timestamp": "N/A",
            "last_send_status": "Chưa gửi"
        }
        # agv_signals_from_pc is used by poll_agv_signals and /api/dashboard_data
        agv_signals_from_pc[agv_id_key] = {
            "signal_received": "N/A",
            "last_received_timestamp": "N/A",
            "last_receive_status": "Chưa nhận"
        }

    # Start AGV polling thread
    polling_thread = threading.Thread(target=poll_agv_signals, daemon=True)
    polling_thread.start()

    ui_host = PC_CONFIG.get("ui_host", "0.0.0.0")
    ui_port = PC_CONFIG.get("ui_port", 7000)

    # The /api/upstream_receive endpoint runs on the same Flask app instance,
    # so it listens on ui_host and ui_port.
    actual_upstream_listener_host = ui_host
    actual_upstream_listener_port = ui_port

    # These are the upstream host/port values from the configuration file.
    configured_upstream_host = PC_CONFIG.get("upstream_host", "0.0.0.0")
    configured_upstream_port = PC_CONFIG.get("upstream_port", 6000)

    print(f"PC Hub UI sẽ chạy trên http://{ui_host}:{ui_port}")
    print(f"PC Hub endpoint /api/upstream_receive lắng nghe trên http://{actual_upstream_listener_host}:{actual_upstream_listener_port}")
    log_pc_event("info", f"PC Hub UI bắt đầu trên http://{ui_host}:{ui_port}")
    log_pc_event("info", f"PC Hub endpoint /api/upstream_receive lắng nghe trên http://{actual_upstream_listener_host}:{actual_upstream_listener_port}")

    # Note: Flask's development server is single-threaded by default for handling requests.
    # For a production environment, you'd use a WSGI server like Gunicorn or Waitress.
    # The polling thread runs independently.
    # To run two Flask apps (one for UI, one for upstream) on different ports within the same script
    # is tricky with the standard `app.run()`. A more robust solution might involve:
    # 1. Running them as separate processes.
    # 2. Using a more advanced setup like threading Flask apps (complex) or a reverse proxy.
    # The current code runs one Flask app serving both UI and the /api/upstream_receive endpoint on `ui_port`.
    if configured_upstream_port != actual_upstream_listener_port or configured_upstream_host != actual_upstream_listener_host:
        warning_msg = (
            f"CẢNH BÁO: Cấu hình upstream host/port ({configured_upstream_host}:{configured_upstream_port}) "
            f"khác với host/port thực tế của endpoint /api/upstream_receive ({actual_upstream_listener_host}:{actual_upstream_listener_port}). "
            "Endpoint này hiện đang được phục vụ trên host/port của UI. "
            "Để có listener riêng cho cấu hình upstream, cần chạy một instance Flask riêng hoặc dùng reverse proxy."
        )
        print(warning_msg)
        log_pc_event("warn", warning_msg)

    app.run(host=ui_host, port=ui_port, debug=False) # debug=False for more stable threading in dev