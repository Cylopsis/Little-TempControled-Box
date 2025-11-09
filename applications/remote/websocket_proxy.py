import asyncio
import json
import websockets

# 板子的TCP服务器地址和端口
TCP_SERVER_IP = "192.168.5.68"
TCP_SERVER_PORT = 5000

# WebSocket服务器的地址和端口
WS_SERVER_IP = "0.0.0.0"
WS_SERVER_PORT = 8765

# 全局共享资源
clients = set()
tcp_writer = None

async def tcp_communication_manager():
    """
    维持一个到TCP服务器的持久连接
    """
    global tcp_writer
    buffer = ""
    while True:
        try:
            reader, writer = await asyncio.open_connection(TCP_SERVER_IP, TCP_SERVER_PORT)
            tcp_writer = writer
            print(f"Successfully connected to TCP server at {TCP_SERVER_IP}:{TCP_SERVER_PORT}")

            # 启动一个独立的任务来定期请求状态
            asyncio.create_task(request_status_periodically())

            while True:
                data = await reader.read(1024)
                if not data:
                    print("TCP server closed the connection. Reconnecting...")
                    tcp_writer = None
                    break
                
                buffer += data.decode('utf-8', errors='ignore')

                # 处理缓冲区中所有完整的消息
                while '\r\n' in buffer:
                    message, buffer = buffer.split('\r\n', 1)
                    try:
                        json_start = message.find('{')
                        if json_start != -1:
                            json_message = message[json_start:]
                            status = json.loads(json_message)
                            
                            if status and clients:
                                broadcast_message = json.dumps(status)
                                await asyncio.gather(
                                    *[client.send(broadcast_message) for client in clients if client.open]
                                )
                    except json.JSONDecodeError as e:
                        # 忽略无法解析的行，因为它们可能是命令的响应而不是状态JSON
                        print(f"Ignoring non-JSON message or parse error: {e}, Message: '{message}'")
                        pass

        except (ConnectionRefusedError, OSError) as e:
            print(f"Failed to connect to TCP server: {e}. Retrying in 5 seconds...")
            tcp_writer = None
            await asyncio.sleep(5)
        except Exception as e:
            print(f"An unexpected error in TCP manager: {e}. Retrying in 5 seconds...")
            tcp_writer = None
            await asyncio.sleep(5)

async def request_status_periodically():
    """定期通过共享的writer发送get_status命令。"""
    while True:
        if tcp_writer and not tcp_writer.is_closing():
            try:
                tcp_writer.write(b"get_status\r\n")
                await tcp_writer.drain()
            except Exception as e:
                print(f"Error sending get_status: {e}")
                # 连接可能已损坏，等待主循环处理重连
                break 
        await asyncio.sleep(0.1)


async def handle_websocket_client(websocket):
    """处理单个WebSocket客户端连接。"""
    clients.add(websocket)
    print(f"New client connected. Total clients: {len(clients)}")
    try:
        async for message in websocket:
            print(f"Received command from client: {message}")
            if tcp_writer and not tcp_writer.is_closing():
                try:
                    tcp_writer.write(message.encode('utf-8') + b'\r\n')
                    await tcp_writer.drain()
                    print("Command sent to TCP server.")
                except Exception as e:
                    print(f"Error sending command to TCP server: {e}")
            else:
                print("Cannot send command: No active TCP connection.")
    except websockets.exceptions.ConnectionClosed:
        print("Client connection closed normally.")
    finally:
        clients.remove(websocket)
        print(f"Client disconnected. Total clients: {len(clients)}")

async def main():
    """主函数，启动WebSocket服务器和TCP通信管理器"""
    asyncio.create_task(tcp_communication_manager())
    
    server = await websockets.serve(handle_websocket_client, WS_SERVER_IP, WS_SERVER_PORT)
    print(f"WebSocket server started at ws://{WS_SERVER_IP}:{WS_SERVER_PORT}")
    
    await server.wait_closed()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped.")
