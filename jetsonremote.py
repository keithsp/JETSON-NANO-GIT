import json
import threading
import time
from typing import Optional

import cv2
import paho.mqtt.client as mqtt
import serial


SERIAL_PORT = "/dev/ttyTHS1"
SERIAL_BAUDRATE = 115200

AIMING_CAMERA_INDEX = 0
AUX_CAMERA_INDEX = 1
FRAME_WIDTH = 640
FRAME_HEIGHT = 490
OBJ_WIDTH = 400
OBJ_HEIGHT = 400

MQTT_BROKER = "192.168.137.1"
MQTT_PORT = 1883
MQTT_TOPIC_COMMAND = "sep3/robot/cmd"
MQTT_TOPIC_CAMERA = "sep3/robot/camera"
MQTT_TOPIC_CAMERA_AUX = "sep3/robot/camera2"
MQTT_TOPIC_TELEMETRY = "sep3/robot/telemetry"
CAMERA_STREAM_FPS = 10.0
CAMERA_JPEG_QUALITY = 70

CONTROL_MSG_LEN = 15
CONTROL_MSG_START = 0x30
CONTROL_MSG_TYPE = 0x01
CONTROL_MSG_END = 0x31

JETSON_STATUS_STX = 0xAA
JETSON_STATUS_PKT_SIZE = 23

ROSM_MOVE_FORWARD = 0x01
ROSM_MOVE_BACKWARD = 0x02
ROSM_MOVE_LEFT = 0x04
ROSM_MOVE_RIGHT = 0x08

ROSM_TURRET_UP = 0x01
ROSM_TURRET_DOWN = 0x02
ROSM_TURRET_LEFT = 0x04
ROSM_TURRET_RIGHT = 0x08

ROSM_AUX_TRIGGER = 0x01
ROSM_AUX_RETRIEVAL_IN = 0x02
ROSM_AUX_RETRIEVAL_OUT = 0x04
ROSM_AUX_AUTO_MODE = 0x08
ROSM_AUX_TARGETING_ENABLE = 0x10

STATE_MANUAL = "MANUAL"
STATE_AUTO_PATROLLING = "AUTO_PATROLLING"
STATE_AUTO_TARGETING = "AUTO_TARGETING"
STATE_ENUM_MAP = {
    0x0: STATE_MANUAL,
    0x1: STATE_AUTO_PATROLLING,
    0x2: STATE_AUTO_TARGETING,
}

MQTT_COMMAND_ALIASES = {
    "W": "FORWARD",
    "UP": "FORWARD",
    "FORWARD": "FORWARD",
    "S": "BACKWARD",
    "DOWN": "BACKWARD",
    "BACKWARD": "BACKWARD",
    "A": "LEFT",
    "LEFT": "LEFT",
    "D": "RIGHT",
    "RIGHT": "RIGHT",
    "STOP": "STOP",
    "NONE": "STOP",
}

MOVEMENT_PRIORITY = (
    ("forward", "FORWARD"),
    ("backward", "BACKWARD"),
    ("left", "LEFT"),
    ("right", "RIGHT"),
)

TURRET_PRIORITY = (
    ("up", "TURRET_UP"),
    ("down", "TURRET_DOWN"),
    ("left", "TURRET_LEFT"),
    ("right", "TURRET_RIGHT"),
)

RETRIEVAL_PRIORITY = (
    ("in", "RETRIEVAL_IN"),
    ("out", "RETRIEVAL_OUT"),
)


class CommandState:
    def __init__(self):
        self._lock = threading.Lock()
        self._state = {
            "movement_bits": 0,
            "turret_bits": 0,
            "aux_bits": 0,
            "label": "STOP",
            "selected_target_id": None,
        }

    def set(self, value: dict):
        with self._lock:
            self._state = value

    def get(self) -> dict:
        with self._lock:
            return dict(self._state)


def command_to_state(command_name: str) -> dict:
    state = {
        "movement_bits": 0,
        "turret_bits": 0,
        "aux_bits": 0,
        "label": command_name,
        "selected_target_id": None,
    }

    if command_name == "FORWARD":
        state["movement_bits"] = ROSM_MOVE_FORWARD
    elif command_name == "BACKWARD":
        state["movement_bits"] = ROSM_MOVE_BACKWARD
    elif command_name == "LEFT":
        state["movement_bits"] = ROSM_MOVE_LEFT
    elif command_name == "RIGHT":
        state["movement_bits"] = ROSM_MOVE_RIGHT
    elif command_name == "TURRET_UP":
        state["turret_bits"] = ROSM_TURRET_UP
    elif command_name == "TURRET_DOWN":
        state["turret_bits"] = ROSM_TURRET_DOWN
    elif command_name == "TURRET_LEFT":
        state["turret_bits"] = ROSM_TURRET_LEFT
    elif command_name == "TURRET_RIGHT":
        state["turret_bits"] = ROSM_TURRET_RIGHT
    elif command_name == "TRIGGER":
        state["aux_bits"] = ROSM_AUX_TRIGGER
    elif command_name == "RETRIEVAL_IN":
        state["aux_bits"] = ROSM_AUX_RETRIEVAL_IN
    elif command_name == "RETRIEVAL_OUT":
        state["aux_bits"] = ROSM_AUX_RETRIEVAL_OUT
    elif command_name == "AUTO_MODE":
        state["aux_bits"] = ROSM_AUX_AUTO_MODE
    elif command_name == "TARGETING_ENABLE":
        state["aux_bits"] = ROSM_AUX_TARGETING_ENABLE
    return state


def apply_selected_fields(state: dict, selected_target_id) -> dict:
    state["selected_target_id"] = selected_target_id
    return state


def parse_command_text(payload_text: str) -> dict:
    text = payload_text.strip()
    if not text:
        return command_to_state("STOP")

    if text.startswith("{") and text.endswith("}"):
        try:
            data = json.loads(text)
            if isinstance(data, dict):
                return parse_command_payload(data)
        except json.JSONDecodeError:
            return command_to_state("STOP")

    return command_to_state(MQTT_COMMAND_ALIASES.get(text.upper(), "STOP"))


def first_active(group, priority_pairs):
    if not isinstance(group, dict):
        return None
    for key, command_name in priority_pairs:
        if group.get(key):
            return command_name
    return None


def parse_command_payload(data: dict) -> str:
    movement_bits = int(data.get("movement_bits", 0)) & 0xFF
    turret_bits = int(data.get("turret_bits", 0)) & 0xFF
    aux_bits = int(data.get("aux_bits", 0)) & 0xFF
    selected_target_id = data.get("selected_target_id")
    if selected_target_id in {"", "--"}:
        selected_target_id = None
    elif selected_target_id is not None:
        try:
            selected_target_id = int(selected_target_id)
        except (TypeError, ValueError):
            selected_target_id = None

    if movement_bits or turret_bits or aux_bits:
        labels = []
        if movement_bits & ROSM_MOVE_FORWARD:
            labels.append("FORWARD")
        if movement_bits & ROSM_MOVE_BACKWARD:
            labels.append("BACKWARD")
        if movement_bits & ROSM_MOVE_LEFT:
            labels.append("LEFT")
        if movement_bits & ROSM_MOVE_RIGHT:
            labels.append("RIGHT")
        if turret_bits & ROSM_TURRET_UP:
            labels.append("TURRET_UP")
        if turret_bits & ROSM_TURRET_DOWN:
            labels.append("TURRET_DOWN")
        if turret_bits & ROSM_TURRET_LEFT:
            labels.append("TURRET_LEFT")
        if turret_bits & ROSM_TURRET_RIGHT:
            labels.append("TURRET_RIGHT")
        if aux_bits & ROSM_AUX_TRIGGER:
            labels.append("TRIGGER")
        if aux_bits & ROSM_AUX_RETRIEVAL_IN:
            labels.append("RETRIEVAL_IN")
        if aux_bits & ROSM_AUX_RETRIEVAL_OUT:
            labels.append("RETRIEVAL_OUT")
        if aux_bits & ROSM_AUX_AUTO_MODE:
            labels.append("AUTO_MODE")
        if aux_bits & ROSM_AUX_TARGETING_ENABLE:
            labels.append("TARGETING_ENABLE")
        return {
            "movement_bits": movement_bits,
            "turret_bits": turret_bits,
            "aux_bits": aux_bits,
            "label": "+".join(labels) if labels else "STOP",
            "selected_target_id": selected_target_id,
        }

    text_command = str(data.get("command", "")).strip()
    if text_command:
        return apply_selected_fields(
            command_to_state(MQTT_COMMAND_ALIASES.get(text_command.upper(), "STOP")),
            selected_target_id,
        )

    movement_command = first_active(data.get("movement"), MOVEMENT_PRIORITY)
    if movement_command is not None:
        return apply_selected_fields(command_to_state(movement_command), selected_target_id)

    turret_command = first_active(data.get("turret"), TURRET_PRIORITY)
    if turret_command is not None:
        return apply_selected_fields(command_to_state(turret_command), selected_target_id)

    if data.get("trigger"):
        return apply_selected_fields(command_to_state("TRIGGER"), selected_target_id)

    retrieval_command = first_active(data.get("retrieval"), RETRIEVAL_PRIORITY)
    if retrieval_command is not None:
        return apply_selected_fields(command_to_state(retrieval_command), selected_target_id)

    mode = str(data.get("mode", "")).strip().lower()
    if mode == "auto":
        return apply_selected_fields(command_to_state("AUTO_MODE"), selected_target_id)

    return apply_selected_fields(command_to_state("STOP"), selected_target_id)


def build_uart_message(cx: int, cy: int, obj_width: int, obj_height: int, control_state: dict) -> bytes:
    cx = max(-32768, min(32767, int(cx)))
    cy = max(-32768, min(32767, int(cy)))

    msg = bytearray(CONTROL_MSG_LEN)
    msg[0] = CONTROL_MSG_START
    msg[1] = CONTROL_MSG_TYPE

    msg[2] = cx & 0xFF
    msg[3] = (cx >> 8) & 0xFF
    msg[4] = cy & 0xFF
    msg[5] = (cy >> 8) & 0xFF

    msg[6] = obj_width & 0xFF
    msg[7] = (obj_width >> 8) & 0xFF
    msg[8] = obj_height & 0xFF
    msg[9] = (obj_height >> 8) & 0xFF

    msg[10] = int(control_state.get("movement_bits", 0)) & 0xFF
    msg[11] = int(control_state.get("turret_bits", 0)) & 0xFF
    msg[12] = int(control_state.get("aux_bits", 0)) & 0xFF
    msg[13] = 0
    msg[14] = CONTROL_MSG_END
    return bytes(msg)


def calc_status_checksum(packet: bytes) -> int:
    checksum = 0
    for value in packet[:-1]:
        checksum ^= value
    return checksum


def signed_byte_to_int(value: int) -> int:
    return value - 256 if value > 127 else value


def yaw_byte_to_deg(value: int) -> float:
    yaw_deg = (float(value) * 360.0) / 255.0
    if yaw_deg > 180.0:
        yaw_deg -= 360.0
    return round(yaw_deg, 1)


def parse_telemetry_packet(packet: bytes):
    if len(packet) != JETSON_STATUS_PKT_SIZE:
        return None
    if packet[0] != JETSON_STATUS_STX:
        return None
    if calc_status_checksum(packet) != packet[-1]:
        return None

    encoder_raw = packet[1] | (packet[2] << 8)
    packed_status = int(packet[7])
    waypoint_idx = (packed_status >> 4) & 0x0F
    state_enum = packed_status & 0x0F
    beacon1 = packet[8] | (packet[9] << 8)
    beacon2 = packet[10] | (packet[11] << 8)
    beacon3 = packet[12] | (packet[13] << 8)
    usonic1 = packet[14] | (packet[15] << 8)
    usonic2 = packet[16] | (packet[17] << 8)
    usonic3 = packet[18] | (packet[19] << 8)
    usonic4 = packet[20] | (packet[21] << 8)

    return {
        "encoder_deg": round(encoder_raw / 10.0, 1),
        "tilt_deg": int(packet[3]),
        "roll_deg": signed_byte_to_int(packet[4]),
        "pitch_deg": signed_byte_to_int(packet[5]),
        "yaw_deg": yaw_byte_to_deg(packet[6]),
        "waypoint_idx": waypoint_idx,
        "robot_state_code": state_enum,
        "robot_state": STATE_ENUM_MAP.get(state_enum, f"UNKNOWN_{state_enum}"),
        "robot_state_enum": STATE_ENUM_MAP.get(state_enum, f"UNKNOWN_{state_enum}"),
        "beacon_1_cm": int(beacon1),
        "beacon_2_cm": int(beacon2),
        "beacon_3_cm": int(beacon3),
        "us_1_cm": int(usonic1),
        "us_2_cm": int(usonic2),
        "us_3_cm": int(usonic3),
        "us_4_cm": int(usonic4),
        "checksum": int(packet[22]),
        "timestamp": time.time(),
    }


class TelemetryParser:
    def __init__(self):
        self.buffer = bytearray()

    def push(self, chunk: bytes):
        packets = []
        if not chunk:
            return packets

        self.buffer.extend(chunk)
        while len(self.buffer) >= JETSON_STATUS_PKT_SIZE:
            if self.buffer[0] != JETSON_STATUS_STX:
                del self.buffer[0]
                continue

            candidate = bytes(self.buffer[:JETSON_STATUS_PKT_SIZE])
            if calc_status_checksum(candidate) == candidate[-1]:
                packets.append(candidate)
                del self.buffer[:JETSON_STATUS_PKT_SIZE]
                continue

            del self.buffer[0]

        return packets


def get_aruco_detector():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV aruco module is not available. Install opencv-contrib-python.")

    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, "DetectorParameters_create"):
        params = aruco.DetectorParameters_create()
    else:
        params = aruco.DetectorParameters()

    detector = aruco.ArucoDetector(dictionary, params) if hasattr(aruco, "ArucoDetector") else None
    return aruco, dictionary, params, detector


def detect_markers(frame, aruco, dictionary, params, detector):
    if detector is not None:
        corners, ids, _ = detector.detectMarkers(frame)
    else:
        corners, ids, _ = aruco.detectMarkers(frame, dictionary, parameters=params)
    return corners, ids


def encode_camera_frame(frame) -> Optional[bytes]:
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(CAMERA_JPEG_QUALITY)],
    )
    if not ok:
        return None
    return encoded.tobytes()


def build_target_records(marker_corners, marker_ids):
    records = []
    if marker_ids is None or len(marker_ids) == 0:
        return records

    for i, marker_id in enumerate(marker_ids.flatten()):
        corners = marker_corners[i].reshape((4, 2))
        cx = int(float(corners[:, 0].mean()))
        cy = int(float(corners[:, 1].mean()))
        records.append(
            {
                "id": int(marker_id),
                "corners": corners,
                "cx": cx,
                "cy": cy,
            }
        )

    records.sort(key=lambda record: record["id"])
    return records


def choose_active_target(records, selected_target_id):
    if not records:
        return None
    if selected_target_id is not None:
        for record in records:
            if record["id"] == selected_target_id:
                return record
    return records[0]


def main():
    command_state = CommandState()
    telemetry_parser = TelemetryParser()
    last_camera_publish_time = 0.0
    latest_telemetry = {
        "robot_state": STATE_MANUAL,
        "robot_state_enum": STATE_MANUAL,
    }

    def on_connect(client, userdata, flags, reason_code, properties):
        print(f"MQTT connected, reason code: {reason_code}")
        client.subscribe(MQTT_TOPIC_COMMAND)

    def on_message(client, userdata, msg):
        payload_text = msg.payload.decode("utf-8", errors="ignore")
        command_state.set(parse_command_text(payload_text))

    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
    except OSError as exc:
        print(f"MQTT disabled: {exc}")
        mqtt_client = None

    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )

    aiming_cap = cv2.VideoCapture(AIMING_CAMERA_INDEX)
    if not aiming_cap.isOpened():
        ser.close()
        raise RuntimeError("Error: Could not open the aiming camera!")

    aux_cap = cv2.VideoCapture(AUX_CAMERA_INDEX)
    if not aux_cap.isOpened():
        print("Warning: Could not open the auxiliary camera.")
        aux_cap.release()
        aux_cap = None

    aiming_cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    aiming_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if aux_cap is not None:
        aux_cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        aux_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    actual_width = int(aiming_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(aiming_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Aiming camera resolution: {actual_width}x{actual_height}")
    if aux_cap is not None:
        aux_width = int(aux_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        aux_height = int(aux_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Aux camera resolution: {aux_width}x{aux_height}")

    print("Starting ArUco detection + dual-camera MQTT streaming + UART...")
    print("Press 'q' to stop")

    aruco, dictionary, params, detector = get_aruco_detector()

    try:
        while True:
            ok, frame = aiming_cap.read()
            if not ok or frame is None:
                print("Error: Could not grab a frame from the aiming camera!")
                break

            aux_frame = None
            if aux_cap is not None:
                aux_ok, aux_frame = aux_cap.read()
                if not aux_ok or aux_frame is None:
                    aux_frame = None

            incoming = ser.read(ser.in_waiting or 1)
            active_control = command_state.get()

            published_telemetry = []
            for packet in telemetry_parser.push(incoming):
                telemetry = parse_telemetry_packet(packet)
                if telemetry is None:
                    continue
                published_telemetry.append(telemetry)
                latest_telemetry.update(telemetry)

            screen_center = (frame.shape[1] / 2.0, frame.shape[0] / 2.0)

            cv2.circle(frame, (int(screen_center[0]), int(screen_center[1])), 6, (0, 255, 255), -1)
            cv2.line(
                frame,
                (int(screen_center[0] - 20), int(screen_center[1])),
                (int(screen_center[0] + 20), int(screen_center[1])),
                (0, 255, 255),
                2,
            )
            cv2.line(
                frame,
                (int(screen_center[0]), int(screen_center[1] - 20)),
                (int(screen_center[0]), int(screen_center[1] + 20)),
                (0, 255, 255),
                2,
            )

            marker_corners, marker_ids = detect_markers(frame, aruco, dictionary, params, detector)
            target_records = build_target_records(marker_corners, marker_ids)
            selected_target_id = active_control.get("selected_target_id")
            active_target = choose_active_target(target_records, selected_target_id)

            sent_this_frame = False

            if target_records:
                for i, record in enumerate(target_records):
                    corners = record["corners"].astype(int)
                    is_selected = selected_target_id is not None and record["id"] == selected_target_id
                    is_active = active_target is not None and record["id"] == active_target["id"]

                    color = (0, 180, 255)
                    if is_selected and is_active:
                        color = (0, 255, 0)
                    elif is_active:
                        color = (0, 255, 255)
                    elif is_selected:
                        color = (255, 0, 255)

                    for idx in range(4):
                        start = tuple(corners[idx])
                        end = tuple(corners[(idx + 1) % 4])
                        cv2.line(frame, start, end, color, 2)

                    cv2.circle(frame, (record["cx"], record["cy"]), 8, color, -1)
                    label = f"ID:{record['id']}"
                    if is_active:
                        label += " ACTIVE"
                    elif is_selected:
                        label += " SELECTED"
                    cv2.putText(
                        frame,
                        label,
                        (record["cx"] + 10, record["cy"] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        color,
                        2,
                    )

                    if active_target is not None and record["id"] == active_target["id"]:
                        cv2.line(
                            frame,
                            (int(screen_center[0]), int(screen_center[1])),
                            (record["cx"], record["cy"]),
                            color,
                            2,
                        )

                if active_target is not None:
                    packet = build_uart_message(active_target["cx"], active_target["cy"], OBJ_WIDTH, OBJ_HEIGHT, active_control)
                    ser.write(packet)
                    sent_this_frame = True

                info_text = (
                    f"VISIBLE:{','.join(str(record['id']) for record in target_records)} "
                    f"SEL:{selected_target_id if selected_target_id is not None else 'AUTO'} "
                    f"ACT:{active_target['id'] if active_target is not None else '--'}"
                )
                cv2.putText(frame, info_text, (40, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "No markers detected", (40, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Continuous UART fail-safe behavior: send default cx/cy when no target.
            if not sent_this_frame:
                ser.write(build_uart_message(0, 0, OBJ_WIDTH, OBJ_HEIGHT, active_control))

            status_text = (
                f"M:{active_control['movement_bits']:02X} "
                f"T:{active_control['turret_bits']:02X} "
                f"A:{active_control['aux_bits']:02X} "
                f"{active_control['label']} "
                f"SEL:{selected_target_id if selected_target_id is not None else 'AUTO'}"
            )
            cv2.putText(frame, status_text, (40, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 255, 255), 2)

            if mqtt_client is not None:
                telemetry_snapshot = dict(latest_telemetry)
                telemetry_snapshot.update(
                    {
                        "visible_targets": [record["id"] for record in target_records],
                        "target_count": len(target_records),
                        "selected_target_id": selected_target_id,
                        "active_target_id": active_target["id"] if active_target is not None else None,
                    }
                )
                mqtt_client.publish(MQTT_TOPIC_TELEMETRY, json.dumps(telemetry_snapshot))

                now = time.time()
                if now - last_camera_publish_time >= (1.0 / CAMERA_STREAM_FPS):
                    camera_payload = encode_camera_frame(frame)
                    if camera_payload is not None:
                        mqtt_client.publish(MQTT_TOPIC_CAMERA, camera_payload)
                    if aux_frame is not None:
                        aux_payload = encode_camera_frame(aux_frame)
                        if aux_payload is not None:
                            mqtt_client.publish(MQTT_TOPIC_CAMERA_AUX, aux_payload)
                    last_camera_publish_time = now

            cv2.imshow("Jetson Remote - ArUco + MQTT + UART", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        aiming_cap.release()
        if aux_cap is not None:
            aux_cap.release()
        cv2.destroyAllWindows()
        ser.close()
        if mqtt_client is not None:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()


if __name__ == "__main__":
    main()
