package excaliburfrc.robot;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.IOException;

public class TestPath {
  Robot robot;
  State[] states;

  @Test
  public void testPathFollowing() throws IOException {
    if (!HAL.initialize(500, 0)) throw new IllegalStateException("Was unable to initialize");
    robot = new Robot();
    states = new ObjectMapper().readValue(
                            Filesystem.getDeployDirectory().toPath().resolve("forward.wpilib.json").toFile(),
                            State[].class);

    Thread robotThread = new Thread(() -> robot.startCompetition());
    robotThread.start();
    SimHooks.pauseTiming();
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);

  }
}
