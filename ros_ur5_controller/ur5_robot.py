import math
import socket
import time


class UR5Robot:
    """Minimal socket-only driver for a UR5 robot.

    This is a stripped-down driver that communicates with the UR5 real-time
    socket program used by the kg_robot framework. It keeps only the commands
    needed by the ROS controller: Cartesian moves, state reads, and shutdown.
    """

    def __init__(self, host, port, tcp=None, payload=None):
        self.host = host
        self.port = port
        self.tcp = tcp if tcp is not None else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.payload = payload if payload is not None else 0.0
        self.c = None
        self.open = False

        self._connect()
        self.set_tcp(self.tcp)
        self.set_payload(self.payload)

    def _connect(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.host, self.port))
        s.listen(5)
        self.c, self.addr = s.accept()
        self.open = True
        print("Connected to UR5")

    def _format_prog(self, cmd, pose=None, acc=0.5, vel=0.5, t=0.0, r=0.0, wait=True):
        if pose is None:
            pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        wait_flag = 0 if wait else 1
        return "({},{},{},{},{},{},{},{},{},{},{},{}\n".format(
            cmd, *pose, acc, vel, t, r, wait_flag
        )

    def _socket_send(self, prog, expect_reply=True):
        msg = "No message from robot"
        try:
            self.c.send(str.encode(prog))
            if expect_reply:
                msg = bytes.decode(self.c.recv(1024))
                if msg == "No message from robot" or msg == "":
                    print("Robot disconnected")
        except socket.error as e:
            print(f"Socket error: {e}")
        return msg

    def socket_send_no_block(self, prog):
        try:
            self.c.send(str.encode(prog))
        except socket.error as e:
            print(f"Socket error: {e}")

    @staticmethod
    def _parse_pose_list(msg):
        """Parse a bracketed comma-separated list of floats."""
        msg = msg.strip()
        if msg.startswith("("):
            msg = msg[1:]
        if msg.endswith(")"):
            msg = msg[:-1]
        # Some responses contain a leading 'p' marker, e.g. "p[1.0,2.0,...]"
        parts = msg.split("p")
        if len(parts) > 1:
            msg = parts[-1]
        msg = msg.strip("[]")
        if not msg:
            return []
        return [float(x.strip()) for x in msg.split(",") if x.strip()]

    def _decode_msg(self, prog):
        msg = self._socket_send(prog)
        return self._parse_pose_list(msg)

    def close(self):
        if self.open and self.c is not None:
            try:
                prog = self._format_prog(100)
                print(self._socket_send(prog))
                self.c.close()
            except Exception as e:
                print(f"Error during close: {e}")
        self.open = False

    # -------------------------------------------------------------------------
    # UR5 Commands
    # -------------------------------------------------------------------------
    def movel_no_block(self, pose, acc=0.5, vel=0.5, min_time=0.0, radius=0.0):
        """Linear move in Cartesian space without blocking."""
        prog = self._format_prog(2, pose=pose, acc=acc, vel=vel, t=min_time, r=radius, wait=False)
        self.socket_send_no_block(prog)

    def getl(self):
        """Get TCP position [x, y, z, rx, ry, rz] with rotation vector."""
        prog = self._format_prog(10)
        return self._decode_msg(prog)

    def getj(self):
        """Get joint positions [j0..j5] in radians."""
        prog = self._format_prog(11)
        return self._decode_msg(prog)

    def get_forces(self):
        """Get [Fx, Fy, Fz, Tx, Ty, Tz]."""
        prog = self._format_prog(14)
        return self._decode_msg(prog)

    def getlv(self):
        """Get TCP velocity [vx, vy, vz, wx, wy, wz]."""
        prog = self._format_prog(16)
        return self._decode_msg(prog)

    def set_tcp(self, tcp):
        """Set robot tool centre point."""
        self.tcp = tcp
        prog = self._format_prog(20, pose=tcp)
        return self._socket_send(prog)

    def set_payload(self, weight, cog=None):
        """Set payload in kg."""
        if cog is None:
            cog = self.tcp
        prog = self._format_prog(21, pose=cog + [0.0, 0.0, 0.0], acc=weight)
        return self._socket_send(prog)


# -------------------------------------------------------------------------
# Helper conversions between rotation vector and quaternion.
# -------------------------------------------------------------------------
def rotvec_to_quaternion(rx, ry, rz):
    """Convert UR5 rotation vector (angle*axis) to quaternion (x, y, z, w)."""
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-6:
        return (0.0, 0.0, 0.0, 1.0)
    x = rx / angle
    y = ry / angle
    z = rz / angle
    s = math.sin(angle / 2.0)
    c = math.cos(angle / 2.0)
    return (x * s, y * s, z * s, c)


def quaternion_to_rotvec(qx, qy, qz, qw):
    """Convert quaternion (x, y, z, w) to UR5 rotation vector."""
    qw = max(-1.0, min(1.0, qw))
    angle = 2.0 * math.acos(qw)
    if abs(angle) < 1e-6:
        return (0.0, 0.0, 0.0)
    s = math.sqrt(1.0 - qw * qw)
    x = qx / s
    y = qy / s
    z = qz / s
    return (x * angle, y * angle, z * angle)
