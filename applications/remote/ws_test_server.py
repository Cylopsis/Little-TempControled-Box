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
import argparse
import signal
import shlex

try:
    import websockets
except Exception as e:
    print("Missing dependency 'websockets'. Install with: pip install websockets")
    raise


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


GAIN_SCHEDULE = [
    {"temperature": 20.0, "kp": 0.05, "ki": 0.010, "kd": 0.005},
    {"temperature": 30.0, "kp": 0.06, "ki": 0.012, "kd": 0.006},
    {"temperature": 40.0, "kp": 0.07, "ki": 0.015, "kd": 0.007},
    {"temperature": 50.0, "kp": 0.08, "ki": 0.018, "kd": 0.008},
    {"temperature": 60.0, "kp": 0.09, "ki": 0.020, "kd": 0.009},
]


DEFAULT_FF_TABLE = [
    {"temperature": 20.0, "base_speed": 0.10},
    {"temperature": 30.0, "base_speed": 0.10},
    {"temperature": 40.0, "base_speed": 0.10},
    {"temperature": 50.0, "base_speed": 0.10},
    {"temperature": 60.0, "base_speed": 0.10},
]


def interpolate_pid(target_c):
    lower = None
    upper = None
    for profile in GAIN_SCHEDULE:
        if profile["temperature"] <= target_c:
            lower = profile
        if profile["temperature"] >= target_c and upper is None:
            upper = profile

    if lower is None:
        return GAIN_SCHEDULE[0]["kp"], GAIN_SCHEDULE[0]["ki"], GAIN_SCHEDULE[0]["kd"]
    if upper is None or lower is upper:
        return lower["kp"], lower["ki"], lower["kd"]

    span = upper["temperature"] - lower["temperature"]
    if span == 0:
        return lower["kp"], lower["ki"], lower["kd"]
    ratio = (target_c - lower["temperature"]) / span
    kp = lower["kp"] + ratio * (upper["kp"] - lower["kp"])
    ki = lower["ki"] + ratio * (upper["ki"] - lower["ki"])
    kd = lower["kd"] + ratio * (upper["kd"] - lower["kd"])
    return kp, ki, kd


def interpolate_feedforward(table, target_c):
    lower = None
    upper = None
    for entry in table:
        if entry["temperature"] <= target_c:
            lower = entry
        if entry["temperature"] >= target_c and upper is None:
            upper = entry

    if lower is None:
        return table[0]["base_speed"]
    if upper is None or lower is upper:
        return lower["base_speed"]

    span = upper["temperature"] - lower["temperature"]
    if span == 0:
        return lower["base_speed"]
    ratio = (target_c - lower["temperature"]) / span
    return lower["base_speed"] + ratio * (upper["base_speed"] - lower["base_speed"])


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
    ff_table = [dict(entry) for entry in DEFAULT_FF_TABLE]
    kp, ki, kd = interpolate_pid(target_temperature)
    integral_error = 0.0
    previous_error = 0.0
    feedforward_speed = interpolate_feedforward(ff_table, target_temperature)
    pid_output = 0.0
    last_update = t0

    def build_status():
        fan_speed = clamp(feedforward_speed + pid_output, 0.0, 1.0)
        return {
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

    async def send_console(lines):
        for line in lines:
            print(f"[server] {line}")
        payload = {"console": lines, "timestamp": round(asyncio.get_event_loop().time() - t0, 3)}
        try:
            await websocket.send(json.dumps(payload))
        except Exception:
            pass

    def format_ff_table():
        lines = [
            "--- Feedforward Table ---",
            "Idx | Temp (C) | Base Speed",
            "----|----------|-----------"
        ]
        for idx, entry in enumerate(ff_table):
            lines.append(f"{idx:>3} | {entry['temperature']:<8.1f} | {entry['base_speed']:.4f}")
        return lines

    async def sender():
        nonlocal current_temperature, current_humidity, env_temperature
        nonlocal ptc_state, btm_ptc_state, control_state
        nonlocal pid_output, feedforward_speed, integral_error, previous_error
        nonlocal target_temperature, kp, ki, kd, ff_table, last_update

        while True:
            loop = asyncio.get_event_loop()
            now = loop.time()
            dt = max(now - last_update, 1e-3)
            last_update = now

            # sensor noise and drift
            current_humidity += (random.random() - 0.5) * 0.05
            env_temperature += (random.random() - 0.5) * 0.02

            error = target_temperature - current_temperature

            if error > 0.5:
                ptc_state = "ON"
                control_state = "HEATING"
            elif error < -0.5:
                ptc_state = "OFF"
                control_state = "COOLING"
            else:
                control_state = "IDLE"

            # emulate bottom PTC activity for visuals
            elapsed = now - t0
            btm_ptc_state = "ON" if int(elapsed) % 30 < 10 else "OFF"

            integral_error += error * dt
            integral_error = clamp(integral_error, -200.0, 200.0)
            derivative = (error - previous_error) / dt if dt > 0 else 0.0
            pid_raw = kp * error + ki * integral_error + kd * derivative
            pid_output = clamp(pid_raw, -0.5, 0.5)
            previous_error = error

            feedforward_speed = clamp(interpolate_feedforward(ff_table, target_temperature), 0.0, 1.0)
            fan_speed = clamp(feedforward_speed + pid_output, 0.0, 1.0)

            # simple plant model: heat plus fan cooling plus environment pull
            heater_effect = 0.12 if ptc_state == "ON" else -0.05
            fan_effect = -0.15 * fan_speed
            ambient_pull = (env_temperature - current_temperature) * 0.02
            noise = (random.random() - 0.5) * 0.04
            current_temperature += (heater_effect + fan_effect + ambient_pull + noise) * dt
            current_temperature += clamp(error * 0.08, -0.25, 0.25) * dt
            current_temperature = clamp(current_temperature, -10.0, 120.0)
            current_humidity = clamp(current_humidity, 0.0, 100.0)

            msg = build_status()

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
            stripped = message.strip()
            if not stripped:
                continue

            if stripped.lower().startswith("get_status"):
                await websocket.send(json.dumps(build_status()))
                continue

            tokens = shlex.split(stripped)
            if not tokens:
                continue

            if tokens[0] != "pid_tune":
                await send_console([f"Error: Unknown command '{tokens[0]}'"])
                continue

            args = tokens[1:]
            if not args:
                await send_console([
                    "--- Usage ---",
                    "pid_tune -t <val>",
                    "pid_tune -p <val> -i <val> -d <val>",
                    "pid_tune -ff",
                    "pid_tune -ff_set <idx> <temp> <spd>"
                ])
                await websocket.send(json.dumps(build_status()))
                continue

            if args[0] == "-ff":
                await send_console(format_ff_table())
                continue

            if args[0] == "-ff_set":
                if len(args) != 4:
                    await send_console(["Error: Usage pid_tune -ff_set <idx> <temp> <spd>"])
                    continue
                try:
                    index = int(args[1])
                    temp_val = float(args[2])
                    speed_val = float(args[3])
                except ValueError:
                    await send_console(["Error: Invalid numeric value in -ff_set"])
                    continue

                if index < 0 or index >= len(ff_table):
                    await send_console([f"Error: Index {index} out of bounds (0-{len(ff_table)-1})."])
                    continue

                ff_table[index]["temperature"] = temp_val
                ff_table[index]["base_speed"] = clamp(speed_val, 0.0, 1.0)
                await send_console([f"FF table entry {index} updated: Temp={temp_val:.1f}C, Speed={speed_val:.4f}"])
                feedforward_speed = interpolate_feedforward(ff_table, target_temperature)
                continue

            # remaining options should come in pairs
            if len(args) % 2 != 0:
                await send_console(["Error: Expected option/value pairs."])
                continue

            target_changed = False
            manual_override = False
            idx = 0
            parse_error = None
            while idx < len(args):
                flag = args[idx]
                value = args[idx + 1]
                try:
                    if flag == "-p":
                        kp = float(value)
                        manual_override = True
                    elif flag == "-i":
                        ki = float(value)
                        manual_override = True
                    elif flag == "-d":
                        kd = float(value)
                        manual_override = True
                    elif flag == "-t":
                        target_temperature = float(value)
                        target_changed = True
                    else:
                        parse_error = f"Error: Unknown option {flag}"
                        break
                except ValueError:
                    parse_error = f"Error: Invalid value for {flag}: {value}"
                    break
                idx += 2

            if parse_error:
                await send_console([parse_error])
                continue

            if target_changed:
                kp, ki, kd = interpolate_pid(target_temperature)
                await send_console(["Target temperature changed. Re-scheduling PID gains..."])

            if manual_override and not target_changed:
                await send_console(["PID gains updated via manual override."])

            feedforward_speed = interpolate_feedforward(ff_table, target_temperature)
            await send_console(["Parameters updated. Current status:"])
            await websocket.send(json.dumps(build_status()))
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        sender_task.cancel()
        try:
            await sender_task
        except asyncio.CancelledError:
            pass
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
