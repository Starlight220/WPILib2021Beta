package excaliburfrc.robot;

import static org.junit.Assert.fail;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import java.io.IOException;
import org.junit.BeforeClass;
import org.junit.Test;

public class TestPath {
  Robot robot;
  State[] states;

  @BeforeClass
  public void setup() {
    robot = new Robot();
    try {
      states =
          new ObjectMapper()
              .readValue(
                  Filesystem.getDeployDirectory().toPath().resolve("forward.wpilib.json").toFile(),
                  State[].class);
    } catch (IOException e) {
      e.printStackTrace();
      fail(e.getMessage());
    }
    robot.startCompetition();
  }

  @Test
  public void testPathFollowing() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
  }
}
