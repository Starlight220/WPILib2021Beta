package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
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

  @Test
  @ResourceLock("timing")
  void testTest() {
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
    pauseTiming();
    for (int i = 0; i < 100; i++) {
      stepTiming(.02);
//      assertEquals(1 + i, DriveSubsystem.simPerCounter.get());
    }

    var pose = robot.m_robotContainer.getRobotDrive().getPose();
    assertAll(
        () -> {
          var x = pose.getX();
          //              assertTrue(0 < x && x < 10);
          assertEquals(5, x, 5);
          assertNotEquals(0, x);
          assertNotEquals(10, x);
        },
        () -> {
          var y = pose.getY();
          assertEquals(5, y, 5);
          assertNotEquals(0, y);
          assertNotEquals(10, y);
        },
        () -> {
          var theta = pose.getRotation().getDegrees();
          assertEquals(0, theta, 90);
          //              assertTrue(-90 <= theta && theta < 90);
        }
        //            () -> assertNotEquals(0, pose.getY(), 0.1),
        //            () -> assertNotEquals(0, pose.getRotation().getDegrees(), 0.1)
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
