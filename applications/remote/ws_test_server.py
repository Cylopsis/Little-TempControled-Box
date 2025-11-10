#!/usr/bin/env python3
"""
Simple WebSocket test server to simulate the temperature-chamber board JSON for offline front-end testing.

Usage:
    python ws_test_server.py [--host HOST] [--port PORT] [--interval SECONDS]

Defaults:
    HOST: 0.0.0.0
    PORT: 8765
    INTERVAL: 0.5

The server broadcasts a periodic JSON object with fields used by the front-end, and prints any commands
received from the client (the front-end sends `pid_tune ...` commands which this server will echo to stdout).

Install dependency:
    pip install websockets

"""

import asyncio
import json
import random
import math
import argparse
import signal

try:
    import websockets
except Exception as e:
    print("Missing dependency 'websockets'. Install with: pip install websockets")
    raise


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


async def client_handler(websocket, path, interval):
    print(f"Client connected: {websocket.remote_address}")

    # initial simulated state
    t0 = asyncio.get_event_loop().time()
    current_temperature = 25.0
    target_temperature = 37.0
    current_humidity = 50.0
    env_temperature = 22.0
    ptc_state = "OFF"
    btm_ptc_state = "OFF"
    control_state = "IDLE"
    kp = 0.06
    ki = 0.012
    kd = 0.006
    integral_error = 0.0
    previous_error = 0.0
    feedforward_speed = 0.10
    pid_output = 0.0

    async def sender():
        nonlocal current_temperature, current_humidity, env_temperature
        nonlocal ptc_state, btm_ptc_state, control_state
        nonlocal pid_output, feedforward_speed, integral_error, previous_error

        while True:
            t = asyncio.get_event_loop().time() - t0
            # random walk + small sinusoidal to feel alive
            current_temperature += (random.random() - 0.5) * 0.05 + math.sin(t / 8.0) * 0.02
            current_humidity += (random.random() - 0.5) * 0.02
            env_temperature += (random.random() - 0.5) * 0.01

            # simple control heuristics for demo
            if current_temperature < target_temperature - 0.5:
                ptc_state = "ON"
                control_state = "HEATING"
            elif current_temperature > target_temperature + 0.5:
                ptc_state = "OFF"
                control_state = "COOLING"
            else:
                control_state = "IDLE"

            # bottom PTC toggles occasionally
            if int(t) % 30 < 10:
                btm_ptc_state = "ON"
            else:
                btm_ptc_state = "OFF"

            # simple feedforward and PID mock
            feedforward_speed = clamp(0.10 + (target_temperature - current_temperature) * 0.01, 0.0, 1.0)
            pid_output = clamp((target_temperature - current_temperature) * (-0.02), -0.5, 0.5)
            fan_speed = clamp(feedforward_speed + pid_output, 0.0, 1.0)

            msg = {
                "current_temperature": round(current_temperature, 2),
                "target_temperature": round(target_temperature, 2),
                "current_humidity": round(current_humidity, 1),
                "env_temperature": round(env_temperature, 2),
                "ptc_state": ptc_state,
                "btm_ptc_state": btm_ptc_state,
                "control_state": control_state,
                "fan_speed": round(fan_speed, 4),
                "fan_speed_percent": round(fan_speed * 100.0, 2),
                "feedforward_speed": round(feedforward_speed, 4),
                "pid_output": round(pid_output, 4),
                "pid_kp": round(kp, 6),
                "pid_ki": round(ki, 6),
                "pid_kd": round(kd, 6),
                "integral_error": round(integral_error, 4),
                "previous_error": round(previous_error, 4)
            }

            try:
                await websocket.send(json.dumps(msg))
            except Exception as e:
                # connection likely closed
                return

            await asyncio.sleep(interval)

    sender_task = asyncio.create_task(sender())

    try:
        async for message in websocket:
            # print commands sent from frontend
            print(f"[client -> server] {message}")
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        sender_task.cancel()
        print(f"Client disconnected: {websocket.remote_address}")


async def main(host, port, interval):
    print(f"Starting WebSocket test server on ws://{host}:{port} (interval={interval}s)")

    async with websockets.serve(lambda ws, path: client_handler(ws, path, interval), host, port):
        # keep running until interrupted
        stop = asyncio.Future()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                asyncio.get_event_loop().add_signal_handler(sig, stop.set_result, None)
            except NotImplementedError:
                # add_signal_handler may not be implemented on Windows event loop
                pass
        await stop


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='WebSocket test server for Temperature Chamber front-end')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8765, help='Port to bind (default: 8765)')
    parser.add_argument('--interval', type=float, default=0.5, help='Send interval in seconds (default: 0.5)')
    args = parser.parse_args()

    try:
        asyncio.run(main(args.host, args.port, args.interval))
    except KeyboardInterrupt:
        print('\nServer stopped by user')
