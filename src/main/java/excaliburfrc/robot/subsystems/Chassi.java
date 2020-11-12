package excaliburfrc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;
import static excaliburfrc.robot.subsystems.Ports.ChassiPorts.*;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassi extends SubsystemBase {
  private CANSparkMax leftLeader;
  private CANSparkMax leftFollower;
  private CANSparkMax rightLeader;
  private CANSparkMax rightFollower;
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  private AHRS gyro;

  private DifferentialDrive drive;
  private DifferentialDriveOdometry odometry;

  public Chassi() {
    leftLeader = new CANSparkMax(LEFT_LEADER, kBrushless);
    leftFollower = new CANSparkMax(LEFT_FOLLOWER, kBrushless);
    leftFollower.follow(leftLeader);

    rightLeader = new CANSparkMax(RIGHT_LEADER, kBrushless);
    rightFollower = new CANSparkMax(RIGHT_FOLLOWER, kBrushless);
    rightFollower.follow(rightLeader);

    leftEncoder = new Encoder(LEFT_ENCODER_A, LEFT_ENCODER_B, false);
    leftEncoder.setDistancePerPulse(GEARING);
    rightEncoder = new Encoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B, true);
    rightEncoder.setDistancePerPulse(GEARING);

    gyro = new AHRS(SPI.Port.kMXP);

    drive = new DifferentialDrive(leftLeader, rightLeader);
    drive.setSafetyEnabled(false);

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0.0));
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(gyro.getAngle()), leftEncoder.get(), rightEncoder.get());
  }

  public Command buildRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
            trajectory,
            odometry::getPoseMeters,
            new RamseteController(),
            new SimpleMotorFeedforward(kV, kS, kA),
            new DifferentialDriveKinematics(TRACK_WIDTH),
            () -> new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate()),
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            (left, right) ->
                drive.tankDrive(
                    RobotController.getBatteryVoltage() * left,
                    RobotController.getBatteryVoltage() * right),
            // (double left, double right)->drive.set,
            this)
        .andThen(drive::stopMotor, this);
  }
}
