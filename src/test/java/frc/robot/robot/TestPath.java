package frc.robot.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestPath {
  Robot robot;

  @BeforeEach
  public void initializeHAL() {
    if (!HAL.initialize(500, 0)) throw new IllegalStateException("Was unable to initialize");
    robot = new Robot();
  }

  @Test
  public void testPathFollowing() {
    Thread robotThread = new Thread(() -> robot.startCompetition());
    robotThread.start();
    SimHooks.pauseTiming();
    SimHooks.stepTiming(0.0);

    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);

    SimHooks.stepTiming(2);
    assertEquals(
        new Pose2d(6, 6, new Rotation2d(0)), robot.m_robotContainer.getRobotDrive().getPose());
    robot.endCompetition();
  }
}
