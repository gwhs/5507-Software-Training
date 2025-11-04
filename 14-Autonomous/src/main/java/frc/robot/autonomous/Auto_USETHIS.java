package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto_USETHIS extends SequentialCommandGroup {
  public Auto_USETHIS() {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");
      PathPlannerPath S_F = PathPlannerPath.fromChoreoTrajectory("Start-F");
      PathPlannerPath F_CS = PathPlannerPath.fromChoreoTrajectory("F-CS");
      PathPlannerPath CS_C = PathPlannerPath.fromChoreoTrajectory("CS-C");
      PathPlannerPath C_CS = PathPlannerPath.fromChoreoTrajectory("C-CS");

      Pose2d startingPose =
          new Pose2d(
            S_F.getPoint(0).position, S_F.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(S_F)
          /*
           * TODO: The rest of the autonomous routine command
           */
          );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
