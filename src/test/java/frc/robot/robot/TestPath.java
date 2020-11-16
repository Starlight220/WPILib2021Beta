package frc.robot.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Robot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.ResourceLock;

import static edu.wpi.first.wpilibj.simulation.SimHooks.pauseTiming;
import static edu.wpi.first.wpilibj.simulation.SimHooks.stepTiming;
import static org.junit.jupiter.api.Assertions.*;

public class TestPath {
  Robot robot;

  @BeforeEach
  public void initializeHAL() {
    if (!HAL.initialize(500, 0)) throw new IllegalStateException("Was unable to initialize");
    robot = new Robot();
  }

  //  @Test
  public void testPathFollowing() {
    Thread robotThread = new Thread(() -> robot.startCompetition());
    robotThread.start();
    pauseTiming();
    stepTiming(0.0);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);

    SimHooks.stepTimingAsync(2);
    assertEquals(
            new Pose2d(6, 6, new Rotation2d(0)), robot.m_robotContainer.getRobotDrive().getPose());
    robot.endCompetition();
  }


  @Test
  @ResourceLock("timing")
  void testTest() {
//    Robot robot = new Robot();

    Thread robotThread = new Thread(() -> robot.startCompetition());
    robotThread.start();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.setProgramStarted();
    assertTrue(SimHooks.getProgramStarted());
    pauseTiming();
    stepTiming(0.0);  // Wait for Notifiers
//    fail();


    assertEquals(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            robot.m_robotContainer.getRobotDrive().getPose());


    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    stepTiming(2.0);
//    resumeTiming();

    var pose = robot.m_robotContainer.getRobotDrive().getPose();
    assertAll(
            () -> assertNotEquals(0, pose.getX(), 0.1),
            () -> assertNotEquals(0, pose.getY(), 0.1),
            () -> assertNotEquals(0, pose.getRotation().getDegrees(), 0.1)

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
