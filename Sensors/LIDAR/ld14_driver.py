import serial
import struct

FRAME_HEADER = b'\x54\x2C'
FRAME_LEN = 47
POINTS_PER_FRAME = 12

class LD14Driver:
    def __init__(self, port, baudrate=230400, timeout=0.05):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.buffer = bytearray()

    def _parse_frame(self, frame):
        # Start and end angles (0.01° units)
        start_angle = struct.unpack('<H', frame[4:6])[0] / 100.0
        end_angle = struct.unpack('<H', frame[42:44])[0] / 100.0

        # Angle step with rollover
        angle_diff = end_angle - start_angle
        if angle_diff < 0:
            angle_diff += 360
        angle_step = angle_diff / (POINTS_PER_FRAME - 1)

        points = []
        offset = 6
        for i in range(POINTS_PER_FRAME):
            distance = struct.unpack('<H', frame[offset:offset+2])[0] / 1000.0
            intensity = frame[offset+2]
            offset += 3

            if distance > 0.05:
                angle_deg = (start_angle + i * angle_step) % 360
                points.append((angle_deg, distance, intensity))

        return points

    def read_frame(self):
        # Read until a full frame is available
        while True:
            self.buffer += self.ser.read(FRAME_LEN)
            while len(self.buffer) >= FRAME_LEN:
                if self.buffer[0:2] == FRAME_HEADER:
                    frame = self.buffer[:FRAME_LEN]
                    self.buffer = self.buffer[FRAME_LEN:]
                    yield self._parse_frame(frame)
                else:
                    # discard bytes until header
                    self.buffer.pop(0)

    def read_rotation(self):
        """Yield (angle_deg, distance_m, intensity) for one full rotation."""
        rotation_points = []
        last_angle = None

        for frame_points in self.read_frame():
            for angle_deg, distance, intensity in frame_points:
                if last_angle is not None and angle_deg < last_angle - 180:
                    # Full rotation completed
                    yield rotation_points
                    rotation_points = []

                rotation_points.append((angle_deg, distance, intensity))
                last_angle = angle_deg
