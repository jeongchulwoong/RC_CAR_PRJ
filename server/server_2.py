from flask import Flask, render_template, request
import socket

app = Flask(__name__)

# 라즈베리파이 서버 주소 및 포트
raspberry_pi_address = '192.168.0.7'
raspberry_pi_port = 5050

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send_data', methods=['POST'])
def send_data():
    button_value = request.form['button_value']
    print("버튼 값 :", button_value)
    # 버튼 값(1, 2, 3)을 라즈베리파이 서버로 전송
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((raspberry_pi_address, raspberry_pi_port))
            print("라즈베리파이에 연결 완료")
            s.send(button_value.encode())
        return 'Success'
    except Exception as e:
        return f'Error: {e}'

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
