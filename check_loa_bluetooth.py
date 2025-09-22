import sounddevice as sd
import path
import os
from support_main.lib_main import edit_csv_tab

path_phan_mem = path.path_phan_mem
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

loa = "JBL Go 3"
data_admin = edit_csv_tab.load_all_stt(path_admin)
for i in range(0,len(data_admin)):
    if len(data_admin[i]) > 1:
        if data_admin[i][0] == "loa":
            loa = data_admin[i][1]

def check_connect_loa():
    devices = sd.query_devices()
    found = False
    for d in devices:
        if loa in d['name']:
            print("✅ Tìm thấy thiết bị âm thanh:", d['name'])
            found = True

    if not found:
        print("❌ Không có JBL Go 3 trong danh sách audio đang active")
    return found

# print(check_connect_loa())
