import json
import threading

import cv2
import paho.mqtt.client as mqtt
import serial


SERIAL_PORT = "/dev/ttyTHS1"
SERIAL_BAUDRATE = 115200

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 490
OBJ_WIDTH = 400
OBJ_HEIGHT = 400

MQTT_BROKER = "192.168.137.1"
MQTT_PORT = 1883
MQTT_TOPIC_COMMAND = "sep3/robot/cmd"

MSG_LEN = 15
MSG_START = 0x30
MSG_TYPE = 0x01
MSG_END = 0x31

# Byte 10: one-byte failsafe command for MCU.
# Bytes 11-12: two-byte command packet.
COMMAND_TABLE = {
    "STOP": (0x00, b"ST"),
    "FORWARD": (0x11, b"FW"),
    "BACKWARD": (0x12, b"BW"),
    "LEFT": (0x13, b"LT"),
    "RIGHT": (0x14, b"RT"),
}

# Accept both WASD and arrow-key style command tokens from MQTT.
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


class CommandState:
    def __init__(self):
        self._lock = threading.Lock()
        self._command = "STOP"

    def set(self, value: str):
        with self._lock:
            self._command = value

    def get(self) -> str:
        with self._lock:
            return self._command


def parse_command_text(payload_text: str) -> str:
    text = payload_text.strip()
    if not text:
        return "STOP"

    # Support JSON payload from dashboard: {"command":"FORWARD", ...}
    if text.startswith("{") and text.endswith("}"):
        try:
            data = json.loads(text)
            if isinstance(data, dict):
                text = str(data.get("command", "")).strip()
        except json.JSONDecodeError:
            return "STOP"

    return MQTT_COMMAND_ALIASES.get(text.upper(), "STOP")


def build_uart_message(cx: int, cy: int, obj_width: int, obj_height: int, command_name: str) -> bytes:
    cx = max(-32768, min(32767, int(cx)))
    cy = max(-32768, min(32767, int(cy)))

    failsafe_byte, cmd2 = COMMAND_TABLE.get(command_name, COMMAND_TABLE["STOP"])

    msg = bytearray(MSG_LEN)
    msg[0] = MSG_START
    msg[1] = MSG_TYPE

    msg[2] = cx & 0xFF
    msg[3] = (cx >> 8) & 0xFF
    msg[4] = cy & 0xFF
    msg[5] = (cy >> 8) & 0xFF

    msg[6] = obj_width & 0xFF
    msg[7] = (obj_width >> 8) & 0xFF
    msg[8] = obj_height & 0xFF
    msg[9] = (obj_height >> 8) & 0xFF

    msg[10] = failsafe_byte
    msg[11] = cmd2[0]
    msg[12] = cmd2[1]
    msg[13] = 0
    msg[14] = MSG_END
    return bytes(msg)


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


def main():
    command_state = CommandState()

    def on_connect(client, userdata, flags, reason_code, properties):
        print(f"MQTT connected, reason code: {reason_code}")
        client.subscribe(MQTT_TOPIC_COMMAND)

    def on_message(client, userdata, msg):
        payload_text = msg.payload.decode("utf-8", errors="ignore")
        command_name = parse_command_text(payload_text)
        command_state.set(command_name)

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
        timeout=1.0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        ser.close()
        raise RuntimeError("Error: Could not open the webcam!")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Actual resolution: {actual_width}x{actual_height}")
    print("Starting ArUco marker detection with center calculation + MQTT command tail...")
    print("Press 'q' to stop")

    aruco, dictionary, params, detector = get_aruco_detector()

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("Error: Could not grab a frame!")
                break

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
            active_command = command_state.get()
            sent_this_frame = False

            if marker_ids is not None and len(marker_ids) > 0:
                aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)

                for i, marker_id in enumerate(marker_ids.flatten()):
                    corners = marker_corners[i].reshape((4, 2))
                    cx = int(float(corners[:, 0].mean()))
                    cy = int(float(corners[:, 1].mean()))

                    cv2.line(
                        frame,
                        (int(screen_center[0]), int(screen_center[1])),
                        (cx, cy),
                        (255, 255, 0),
                        2,
                    )
                    cv2.circle(frame, (cx, cy), 8, (255, 0, 0), -1)

                    packet = build_uart_message(cx, cy, OBJ_WIDTH, OBJ_HEIGHT, active_command)
                    ser.write(packet)
                    sent_this_frame = True

                    info_text = f"ID:{int(marker_id)} Center:({cx},{cy}) CMD:{active_command}"
                    cv2.putText(frame, info_text, (40, 50 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "No markers detected", (40, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Continuous UART fail-safe behavior: send default cx/cy when no target.
            if not sent_this_frame:
                ser.write(build_uart_message(0, 0, OBJ_WIDTH, OBJ_HEIGHT, active_command))

            cv2.putText(frame, f"CMD: {active_command}", (40, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 255, 255), 2)
            cv2.imshow("Jetson Remote - ArUco + MQTT + UART", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        ser.close()
        if mqtt_client is not None:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()


if __name__ == "__main__":
    main()
