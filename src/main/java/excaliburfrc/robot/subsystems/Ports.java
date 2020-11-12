package excaliburfrc.robot.subsystems;

class Ports {
  static class ChassiPorts {
    static final int LEFT_LEADER = 10,
        RIGHT_LEADER = 11,
        LEFT_FOLLOWER = 12,
        RIGHT_FOLLOWER = 13,
        LEFT_ENCODER_A = 0,
        RIGHT_ENCODER_A = 1,
        LEFT_ENCODER_B = 2,
        RIGHT_ENCODER_B = 3;
    static final double GEARING = 1.0,
        TRACK_WIDTH = 0.6, // meters
        MOMENT_OF_INERTIA = 0.6, // kg/meters^2
        MASS = 0.6, // kg
        WHEEL_RADIUS = 0.6, // meters
        kV = 0.1,
        kS = 0.1,
        kA = 0.2;
  }
}
