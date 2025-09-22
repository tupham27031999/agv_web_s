from flask import Flask, request

# Tạo một đối tượng Flask
app = Flask(__name__)

# Định nghĩa một route (đường dẫn) cho trang chủ
# Khi người dùng truy cập vào địa chỉ gốc (/)
@app.route('/')
def hello_world():
    return 'Xin chào, đây là server Python của bạn!'

# Định nghĩa một route để nhận dữ liệu qua cả phương thức GET và POST
# Ví dụ: GET: http://192.168.0.101:5000/api?name=agv1"
#         POST: gửi JSON {'name': 'Jane'}
@app.route('/api', methods=['GET', 'POST'])
def api_handler():
    # Xử lý yêu cầu GET
    if request.method == 'GET':
        name = request.args.get('name', 'bạn')
        print(f'Nhận dữ liệu GET từ người dùng: {name}')
        return f'Chào mừng, {name}! Dữ liệu của bạn đã được nhận qua GET.'
    
    # Xử lý yêu cầu POST
    elif request.method == 'POST':
        # Lấy dữ liệu JSON từ phần thân của yêu cầu
        data = request.json
        print("data", data)
        if data and 'name' in data:
            name = data['name']
            print(f'Nhận dữ liệu POST từ người dùng: {name}')
            return {'message': f'Dữ liệu của bạn đã được nhận qua POST, {name}.'}, 201
        else:
            print('Dữ liệu POST không hợp lệ.')
            return {'error': 'Dữ liệu POST không hợp lệ.'}, 400

# Chạy server
if __name__ == '__main__':
    # debug=True sẽ tự động khởi động lại server khi bạn thay đổi mã
    # Thành dòng này để server lắng nghe tất cả các kết nối đến
    app.run(host='0.0.0.0', debug=True)

