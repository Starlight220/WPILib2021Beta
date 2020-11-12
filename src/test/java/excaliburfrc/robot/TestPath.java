package excaliburfrc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;

public class TestPath {
  Robot robot;
  List<State> states;

  @Test
  public void testPathFollowing() throws IOException {
    if (!HAL.initialize(500, 0)) throw new IllegalStateException("Was unable to initialize");
    robot = new Robot();
    states =
        Arrays.<State>asList(
            new ObjectMapper()
                .readValue(
                    Filesystem.getDeployDirectory()
                        .toPath()
                        .resolve("forward.wpilib.json")
                        .toFile(),
                    State[].class));

    Thread robotThread = new Thread(() -> robot.startCompetition());
    robotThread.start();
    SimHooks.pauseTiming();
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);

    SimHooks.stepTiming(0.5);
    assertEquals(states.get(states.size() - 1).poseMeters, robot.m_drive.getPose());
  }
}
