import serial
import numpy as np
import cv2
import time
import threading

# --- CONFIGURATION ---
# Port 1: High Speed Data (USB CDC)
VIDEO_PORT = 'COM6'
VIDEO_BAUD = 921600
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
FRAME_SIZE = FRAME_WIDTH * FRAME_HEIGHT * 2
HEADER = b'\xAA\x55\xAA\x55'

# Port 2: Command & Control (UART3 from your main.c)
CMD_PORT = 'COM4'
CMD_BAUD = 115200

# --- SHARED STATE ---
running = True
current_framerate_setting = 30  # Default to match STM32 default

# --- SERIAL SETUP ---
try:
    # 1. Open Video Stream
    ser_video = serial.Serial(VIDEO_PORT, VIDEO_BAUD, timeout=2)
    print(f"✅ Video connected on {VIDEO_PORT}")

    # 2. Open Command Port
    ser_cmd = serial.Serial(CMD_PORT, CMD_BAUD, timeout=1)
    print(f"✅ Command link connected on {CMD_PORT}")
except serial.SerialException as e:
    print(f"❌ Serial Error: {e}")
    exit()


# --- HELPER FUNCTIONS ---

def read_exact(ser, n):
    """Read exactly n bytes from the video port."""
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk: return None
        buf.extend(chunk)
    return bytes(buf)


def find_header(ser):
    """Sync to the frame header."""
    buf = bytearray()
    while True:
        b = ser.read(1)
        if not b: return None
        buf += b
        if len(buf) > 4: buf = buf[-4:]
        if buf == HEADER: return True


def read_frame_bytes():
    """Get one frame from video port."""
    if not find_header(ser_video): return None
    data = read_exact(ser_video, FRAME_SIZE)
    return data


def send_command(cmd_str):
    """Send a command to STM32 over the UART port."""
    full_cmd = f"{cmd_str}\n"
    print(f"-> Sending: {cmd_str}")
    ser_cmd.write(full_cmd.encode('utf-8'))


def listen_to_stm32():
    """
    Background thread function.
    Reads printf messages from STM32 (UART3) and prints them to Python console.
    """
    while running:
        try:
            if ser_cmd.in_waiting > 0:
                # Read line, decode, strip whitespace
                line = ser_cmd.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"   [STM32 SAYS]: {line}")
        except Exception as e:
            print(f"Command Port Error: {e}")
        time.sleep(0.01)  # Yield slightly to save CPU


def draw_outlined_text(img, text, x, y, scale=0.6, color=(0, 255, 0)):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


# --- START LISTENER THREAD ---
t = threading.Thread(target=listen_to_stm32)
t.start()

# --- MAIN LOOP ---
# Display Modes
MODE_GRAY = 0
MODE_HSV = 1
MODE_DIFF = 2
display_mode = MODE_GRAY
reference = None

# FPS Logic
last_t = time.time()
frame_count = 0
current_fps = 0.0

print("\n--- CONTROLS ---")
print(" [1] Capture Mode    [2] Simulate Mode")
print(" [c] Calibrate Ref   [m] Toggle View Mode")
print(" [+] Increase FPS    [-] Decrease FPS")
print(" [q] Quit")
print("----------------\n")

try:
    while True:
        # 1. Read Video Frame
        frame_bytes = read_frame_bytes()
        if frame_bytes is None: continue

        # 2. Process Image
        frame_u16 = np.frombuffer(frame_bytes, dtype=np.uint16).reshape(FRAME_HEIGHT, FRAME_WIDTH)

        if display_mode == MODE_GRAY:
            img = (frame_u16 >> 8).astype(np.uint8)
        elif display_mode == MODE_HSV:
            hue = ((frame_u16.astype(np.uint32) * 180) // 65536).astype(np.uint8)
            sat = np.full_like(hue, 255)
            val = np.full_like(hue, 255)
            img = cv2.cvtColor(cv2.merge([hue, sat, val]), cv2.COLOR_HSV2BGR)
        elif display_mode == MODE_DIFF:
            if reference is None:
                img = (frame_u16 >> 8).astype(np.uint8)
            else:
                diff = np.abs(frame_u16.astype(np.int32) - reference.astype(np.int32))
                diff = np.minimum(diff, 65536 - diff)
                img = (diff >> 8).astype(np.uint8)

        if img.ndim == 2:
            disp = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            disp = img.copy()

        # 3. FPS Calc
        frame_count += 1
        now = time.time()
        if now - last_t >= 1.0:
            current_fps = frame_count / (now - last_t)
            frame_count = 0
            last_t = now

        # 4. Draw OSD
        mode_name = ["GRAY", "HSV", "DIFF"][display_mode]
        draw_outlined_text(disp, f"View: {mode_name}", 10, 20)
        draw_outlined_text(disp, f"FPS: {current_fps:.1f}", 10, 45)

        # Display current Requested FPS target
        draw_outlined_text(disp, f"Target FPS: {current_framerate_setting}", 10, 70, color=(0, 255, 255))

        cv2.imshow("STM32 Controller", disp)

        # 5. Handle Input
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('m'):
            display_mode = (display_mode + 1) % 3

        # --- COMMANDS SENT TO STM32 ---
        elif key == ord('1'):
            send_command("MODE_CAPTURE")
        elif key == ord('2'):
            send_command("MODE_SIMULATE")
        elif key == ord('c'):
            send_command("CALIBRATE")
            reference = frame_u16.copy()  # Also update local reference
            print("Local reference updated.")

        elif key == ord('=') or key == ord('+'):  # + key
            current_framerate_setting += 5
            if current_framerate_setting > 120: current_framerate_setting = 120
            send_command(f"MAX_FRAMERATE={current_framerate_setting}")

        elif key == ord('-') or key == ord('_'):  # - key
            current_framerate_setting -= 5
            if current_framerate_setting < 1: current_framerate_setting = 1
            send_command(f"MAX_FRAMERATE={current_framerate_setting}")
        elif key == ord('l'):
            send_command("DATA_LOG")

finally:
    running = False  # Stop the listener thread
    t.join()  # Wait for thread to exit
    ser_video.close()
    ser_cmd.close()
    cv2.destroyAllWindows()
    print("Disconnected.")