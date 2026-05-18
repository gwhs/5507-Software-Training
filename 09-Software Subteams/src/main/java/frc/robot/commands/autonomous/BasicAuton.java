package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.groundIntakePivot.GroundIntakePivotSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class BasicAuton extends SequentialCommandGroup {
  public BasicAuton(
    ShooterSubsystem shooter, 
    IndexerSubsystem indexer, 
    GroundIntakePivotSubsystem groundIntakePivot, 
    GroundIntakeRollerSubsystem groundIntakeRoller) {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */
      PathPlannerPath startingPath = PathPlannerPath.fromChoreoTrajectory("path1");

      addCommands(
          AutoBuilder.resetOdom(startingPath.getStartingHolonomicPose().get())
          /*
           * TODO: The rest of the autonomous routine command
           */

          );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
