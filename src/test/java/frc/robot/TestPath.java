package frc.robot;

import static edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming;
import static edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.ResourceLock;

import java.util.concurrent.CompletableFuture;

public class TestPath {
  Robot robot;

  @BeforeEach
  public void initializeHAL() {
    if (!HAL.initialize(500, 0)) throw new IllegalStateException("Was unable to initialize");
    robot = new Robot();
  }

  @Test
  @ResourceLock("timing")
  void testTest() {
    var done = new CompletableFuture<Boolean>();
    //    Robot robot = new Robot();

    Thread robotThread = new Thread(() -> robot.startCompetition());
    robotThread.start();
    pauseTiming();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.setProgramStarted();
    assertTrue(SimHooks.getProgramStarted());
    stepTiming(0.0); // Wait for Notifiers
    //    fail();

    assertEquals(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        robot.m_robotContainer.getRobotDrive().getPose());

    //    assertEquals(0, DriveSubsystem.simPerCounter.get());

    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);

    var current = CommandScheduler.getInstance().requiring(robot.m_robotContainer.getRobotDrive());


    var pose = robot.m_robotContainer.getRobotDrive().getPose();
    System.err.println(pose);
    assertAll(
            () -> assertEquals(6, pose.getX(), 0.1),
            () -> assertEquals(6, pose.getY(), 0.1),
            () -> assertEquals(0, pose.getRotation().getDegrees(), 1)
            );

    robot.endCompetition();
    try {
      robotThread.interrupt();
      robotThread.join();
    } catch (InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
    robot.close();
  }
}
