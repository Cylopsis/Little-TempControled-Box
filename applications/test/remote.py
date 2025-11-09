import socket
import json

# --- 配置 ---
HOST = '192.168.0.106'  # 替换为你的开发板IP
PORT = 5000

try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"Connecting to {HOST}:{PORT}...")
        s.connect((HOST, PORT))
        print("Connected.")
        
        # 发送命令
        command = "get_status\n"
        print(f"Sending command: {command.strip()}")
        s.sendall(command.encode('utf-8'))
        
        # 接收响应
        response = s.recv(1024).decode('utf-8')
        print("Received response:")
        
        # 尝试解析为JSON并格式化打印
        try:
            data = json.loads(response)
            print(json.dumps(data, indent=4))
        except json.JSONDecodeError:
            print(response)

except ConnectionRefusedError:
    print("Connection refused. Is the server running on the board?")
except Exception as e:
    print(f"An error occurred: {e}")
