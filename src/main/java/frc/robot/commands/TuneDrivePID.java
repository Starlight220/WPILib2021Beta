package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TuneDrivePID extends CommandBase {
  private DriveSubsystem drive;
  private PIDController controller;

  public TuneDrivePID(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(this.drive);
    this.controller = new PIDController(0, 0, 0);
    assert SendableRegistry.remove(controller);
    LiveWindow.disableTelemetry(controller);
    this.controller.setTolerance(0.00000001);
    SmartDashboard.putData("TunePID", this);
  }

  @Override
  public void initialize() {
    drive.resetOdometry(new Pose2d(0, 4, Rotation2d.fromDegrees(0)));
    controller.setSetpoint(16);
    controller.reset();
  }

  @Override
  public void execute() {
    drive.arcadeDrive(controller.calculate(drive.getPose().getX()), 0);
  }

  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("kp", controller::getP, controller::setP);
    builder.addDoubleProperty("ki", controller::getI, controller::setI);
    builder.addDoubleProperty("kd", controller::getD, controller::setD);
    builder.addDoubleProperty("error", controller::getPositionError, null);
  }
}
