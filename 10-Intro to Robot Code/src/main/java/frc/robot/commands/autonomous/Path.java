package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Path extends SequentialCommandGroup {
  public Path() {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath Start_F = PathPlannerPath.fromChoreoTrajectory("Start_F");
      PathPlannerPath F_CS = PathPlannerPath.fromChoreoTrajectory("F_CS");
      PathPlannerPath CS_C = PathPlannerPath.fromChoreoTrajectory("CS_C");
      PathPlannerPath C_CS = PathPlannerPath.fromChoreoTrajectory("C_CS");
      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");

      Pose2d startingPose =
          new Pose2d(
              Start_F.getPoint(0).position, Start_F.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(Start_F),
          AutoBuilder.followPath(F_CS),
          AutoBuilder.followPath(CS_C),
          AutoBuilder.followPath(C_CS)

          /*
           * TODO: The rest of the autonomous routine command
           */
          
          );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
