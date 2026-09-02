import math

import rtde_control
import rtde_receive


class UR5Robot:
    """Driver for a UR5 robot based on the ur_rtde library.

    Unlike the old socket-only driver, this connects to the robot as an RTDE
    client. The robot must be running the official ``ExternalControl`` URCap
    program on the teach pendant, which opens the RTDE data and script ports
    (30003 / 30004).
    """

    def __init__(self, host, tcp=None, payload=None, logger=None):
        self.host = host
        self.tcp = tcp if tcp is not None else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.payload = payload if payload is not None else 0.0
        self.logger = logger
        self.is_speeding = False

        self._connect()

    def _connect(self):
        self.rtde_c = rtde_control.RTDEControlInterface(self.host)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.host)
        if not self.rtde_c.isConnected():
            raise ConnectionError(f"Unable to connect RTDE control to {self.host}")
        if not self.rtde_r.isConnected():
            raise ConnectionError(f"Unable to connect RTDE receive to {self.host}")
        # if self.tcp is not None:
        #     self.rtde_c.setTcp(self.tcp)
        # if self.payload is not None:
        #     self.rtde_c.setPayload(self.payload)

    def close(self):
        if self.is_speeding:
            try:
                self.rtde_c.speedStop()
            except Exception as e:
                if self.logger is not None:
                    self.logger.warning(f"speedStop failed during close: {e}")
            self.is_speeding = False
        try:
            self.rtde_c.stopScript()
        except Exception as e:
            if self.logger is not None:
                self.logger.warning(f"stopScript failed during close: {e}")
        self.rtde_r.disconnect()
        self.rtde_c.disconnect()

    # -------------------------------------------------------------------------
    # UR5 Commands
    # -------------------------------------------------------------------------
    def movel_no_block(self, pose, acc=0.5, vel=0.5, min_time=0.0, radius=0.0):
        """Linear move in Cartesian space without blocking."""
        if self.is_speeding:
            self.rtde_c.speedStop()
            self.is_speeding = False
        self.rtde_c.moveL(pose, vel, acc, radius)

    def speedl(self, velocity, acc=1.0, duration=0.02):
        """Set TCP velocity [vx,vy,vz,wx,wy,wz] and block until it completes."""
        self.is_speeding = True
        return self.rtde_c.speedL(velocity, acc, duration)

    def speedl_no_block(self, velocity, acc=1.0):
        """Set TCP velocity [vx,vy,vz,wx,wy,wz] without blocking."""
        self.is_speeding = True
        return self.rtde_c.speedL(velocity, acc, 0.0)

    def speed_stop(self):
        if self.is_speeding:
            self.rtde_c.speedStop()
            self.is_speeding = False

    def getl(self):
        """Get TCP position [x, y, z, rx, ry, rz] with rotation vector."""
        return list(self.rtde_r.getActualTCPPose())

    def getj(self):
        """Get joint positions [j0..j5] in radians."""
        return list(self.rtde_r.getActualQ())

    def get_forces(self):
        """Get [Fx, Fy, Fz, Tx, Ty, Tz]."""
        return list(self.rtde_r.getActualTCPForce())

    def getlv(self):
        """Get TCP velocity [vx, vy, vz, wx, wy, wz]."""
        return list(self.rtde_r.getActualTCPSpeed())

    def set_tcp(self, tcp):
        """Set robot tool centre point."""
        self.tcp = tcp
        return self.rtde_c.setTcp(tcp)

    def set_payload(self, weight, cog=None):
        """Set payload in kg."""
        self.payload = weight
        if cog is None:
            cog = self.tcp
        return self.rtde_c.setPayload(weight, cog)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


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
