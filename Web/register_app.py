import flask
from flask import request, render_template, jsonify
import cv2
import face_recognition
import pickle
import os
import numpy as np
import base64
import re
import unicodedata # Thư viện để dọn dẹp tên file

# Khởi tạo Flask App
app = flask.Flask(__name__,)

# Thư mục để lưu các file .pkl
ENCODINGS_DIR = "Register-ID"

def sanitize_filename(name):
    """Dọn dẹp chuỗi để tạo tên file an toàn"""
    name = unicodedata.normalize('NFKD', name).encode('ascii', 'ignore').decode('ascii')
    name = re.sub(r'[^\w\s-]', '', name).strip()
    name = re.sub(r'[-\s]+', '_', name)
    return name

@app.route('/')
def index():
    """Hiển thị trang web đăng ký"""
    return render_template('register.html')

@app.route('/register', methods=['POST'])
def register_face():
    """API để nhận và xử lý ảnh đăng ký"""
    try:
        # Lấy tên và mã NV
        name = request.form['name']
        employee_id = request.form['employee_id']
        image_data_url = request.form['image']
        
        # Tạo thư mục nếu chưa có
        os.makedirs(ENCODINGS_DIR, exist_ok=True)
        
        # Tạo tên file duy nhất (ví dụ: Tran_Van_Tin_NV123.pkl)
        safe_name = sanitize_filename(name)
        safe_id = sanitize_filename(employee_id)
        filename = f"{safe_name}_{safe_id}.pkl"
        filepath = os.path.join(ENCODINGS_DIR, filename)

        # Xử lý ảnh (Base64 -> CV2 Image)
        image_data_base64 = re.sub('^data:image/.+;base64,', '', image_data_url)
        img_bytes = base64.b64decode(image_data_base64)
        nparr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Tìm khuôn mặt
        face_locations = face_recognition.face_locations(img_rgb, model="hog") # 'hog' nhanh hơn

        if len(face_locations) == 0:
            return jsonify({"status": "error", "message": "Lỗi: Không tìm thấy khuôn mặt nào."})
        if len(face_locations) > 1:
            return jsonify({"status": "error", "message": "Lỗi: Tìm thấy nhiều hơn 1 khuôn mặt."})
        
        # Lấy encoding (đặc trưng 128-số)
        encoding = face_recognition.face_encodings(img_rgb, face_locations)[0]

        # --- LOGIC LƯU FILE (Hỗ trợ nhiều ảnh) ---
        
        if os.path.exists(filepath):
            # 1. File đã tồn tại -> Tải file cũ
            with open(filepath, 'rb') as f:
                data_to_save = pickle.load(f)
            
            # Thêm encoding MỚI vào danh sách
            data_to_save["encodings"].append(encoding)
            msg = f"Đã thêm ảnh mới cho: {name}. Tổng cộng: {len(data_to_save['encodings'])} ảnh."
            
        else:
            # 2. File chưa tồn tại -> Tạo mới
            data_to_save = {
                "name": name, 
                "encodings": [encoding] # Tạo danh sách mới với encoding đầu tiên
            }
            msg = f"Đăng ký thành công (ảnh đầu tiên) cho: {name}!"

        # 3. Lưu (ghi đè) file .pkl
        with open(filepath, 'wb') as f:
            pickle.dump(data_to_save, f)
        # --- KẾT THÚC LOGIC LƯU FILE ---

        return jsonify({"status": "success", "message": msg})

    except Exception as e:
        print(f"Lỗi nghiêm trọng: {e}")
        return jsonify({"status": "error", "message": f"Lỗi server: {e}"})

if __name__ == '__main__':
    print(f"Server đăng ký đang chạy tại http://0.0.0.0:5002")
    print(f"Dữ liệu khuôn mặt sẽ được lưu vào thư mục: {ENCODINGS_DIR}/")
    app.run(host='0.0.0.0', port=5002, debug=False)