package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.EagleUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Test extends SequentialCommandGroup {
  public Test(
      RobotContainer robotContainer,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      EndEffectorSubsystem endEffector,
      SwerveSubsystem drivetrain) {

    /* All your code should go inside this try-catch block */
    try {

      /*
        TODO: Load Paths
      */

      // PathPlannerPath another_path = PathPlannerPath.fromChoreoTrajectory("PATH NAME");
      PathPlannerPath Start_F = PathPlannerPath.fromChoreoTrajectory("Start-F");
      PathPlannerPath F_CS = PathPlannerPath.fromChoreoTrajectory("F-CS");
      PathPlannerPath CS_C = PathPlannerPath.fromChoreoTrajectory("CS-C");
      PathPlannerPath C_CS = PathPlannerPath.fromChoreoTrajectory("C-CS");
      PathPlannerPath CS_D = PathPlannerPath.fromChoreoTrajectory("CS-D");
      PathPlannerPath D_CS = PathPlannerPath.fromChoreoTrajectory("D-CS");
      PathPlannerPath CS_E = PathPlannerPath.fromChoreoTrajectory("CS-E");
      PathPlannerPath E_CS = PathPlannerPath.fromChoreoTrajectory("E-CS");
      Pose2d startingPose =
          new Pose2d(E_CS.getPoint(0).position, E_CS.getIdealStartingState().rotation());
      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          /*
           * TODO: The rest of the autonomous routine command
           */
          AutoBuilder.followPath(E_CS),
          AutoBuilder.followPath(CS_C)
              .deadlineFor(
                  Commands.sequence(
                      Commands.waitSeconds(2.0 - 1.3),
                      robotContainer.prepScoreCoral(
                          ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION)))
              .raceWith(
                  Commands.waitSeconds(0.5)
                      .andThen(
                          Commands.idle()
                              .onlyIf(
                                  () ->
                                      endEffector
                                          .hasGamePiece()))), // Interrupt if the coral does not
          // have piece
          drivetrain
              .alignToPose(() -> EagleUtil.getClosestCoralStation(drivetrain.getState().Pose))
              .withTimeout(2.5)
              .deadlineFor(
                  robotContainer.prepScoreCoral(
                      ElevatorConstants.INTAKE_METER, ArmConstants.ARM_INTAKE_ANGLE)),
          AutoBuilder.followPath(CS_C)
              .deadlineFor(
                  Commands.sequence(
                      Commands.waitSeconds(2.0 - 1.3),
                      robotContainer.prepScoreCoral(
                          ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION)))
              .raceWith(
                  Commands.waitSeconds(0.5)
                      .andThen(
                          Commands.idle()
                              .onlyIf(
                                  () ->
                                      endEffector
                                          .hasGamePiece()))), // Interrupt if the coral does not
          // have piece
          drivetrain
              .alignToPose(() -> EagleUtil.getClosestCoralStation(drivetrain.getState().Pose))
              .withTimeout(2.5)
              .deadlineFor(
                  robotContainer.prepScoreCoral(
                      ElevatorConstants.INTAKE_METER, ArmConstants.ARM_INTAKE_ANGLE)));
    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
