package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.EagleUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorConstants;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Auto_Coral_Return extends SequentialCommandGroup {
  public Auto_Coral_Return(
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

      PathPlannerPath Start_F = PathPlannerPath.fromChoreoTrajectory("Start-F");
      PathPlannerPath F_CS = PathPlannerPath.fromChoreoTrajectory("F-CS");
      PathPlannerPath CS_C = PathPlannerPath.fromChoreoTrajectory("CS-C");
      PathPlannerPath C_CS = PathPlannerPath.fromChoreoTrajectory("C-CS");
      PathPlannerPath CS_D = PathPlannerPath.fromChoreoTrajectory("CS-D");
      PathPlannerPath D_CS = PathPlannerPath.fromChoreoTrajectory("D-CS");
      PathPlannerPath CS_E = PathPlannerPath.fromChoreoTrajectory("CS-E");
      PathPlannerPath E_CS = PathPlannerPath.fromChoreoTrajectory("E-CS");
      Pose2d startingPose =
          new Pose2d(Start_F.getPoint(0).position, Start_F.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          /*
           * TODO: The rest of the autonomous routine command
           */
          Commands.sequence(
              Commands.sequence(
                  AutoBuilder.followPath(Start_F)
                      .deadlineFor(
                          robotContainer.prepScoreCoral(
                              ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION)),
                  endEffector.runVoltage(EndEffectorConstants.VOLTAGE_L4),
                  AutoBuilder.followPath(F_CS)
                      .deadlineFor(
                          robotContainer.prepIntake(
                              ElevatorConstants.INTAKE_METER, ArmConstants.ARM_INTAKE_ANGLE))),
              autoHelper(CS_C, C_CS, robotContainer, drivetrain, endEffector),
              autoHelper(CS_D, D_CS, robotContainer, drivetrain, endEffector),
              autoHelper(CS_E, E_CS, robotContainer, drivetrain, endEffector)));
    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }

  public Command autoHelper(
      PathPlannerPath pathOne,
      PathPlannerPath pathTwo,
      RobotContainer robotContainer,
      SwerveSubsystem drivetrain,
      EndEffectorSubsystem endEffector) {
    return Commands.sequence(
        AutoBuilder.followPath(pathOne)
            .raceWith(
                Commands.waitSeconds(0.3)
                    .andThen(Commands.idle().onlyIf(() -> endEffector.hasGamePiece())))
            .deadlineFor(
                Commands.sequence(
                    Commands.waitSeconds(0.3),
                    robotContainer.prepScoreCoral(
                        ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION))),
        Commands.sequence(
                Commands.parallel(
                        drivetrain.alignToPose(
                            () -> EagleUtil.getClosestCoralStation(drivetrain.getState().Pose)),
                        robotContainer.prepIntake(
                            ElevatorConstants.INTAKE_METER, ArmConstants.ARM_INTAKE_ANGLE))
                    .withTimeout(0.4),
                Commands.waitUntil(() -> endEffector.hasGamePiece()),
                AutoBuilder.followPath(pathOne)
                    .deadlineFor(
                        Commands.sequence(
                            Commands.waitSeconds(0.7),
                            robotContainer.prepScoreCoral(
                                ElevatorConstants.L4_PREP_POSITION,
                                ArmConstants.L4_PREP_POSITION))))
            .onlyIf(() -> !endEffector.hasGamePiece()),
        endEffector.runVoltage(EndEffectorConstants.VOLTAGE_L4),
        AutoBuilder.followPath(pathTwo)
            .deadlineFor(
                robotContainer.prepIntake(
                    ElevatorConstants.INTAKE_METER, ArmConstants.ARM_INTAKE_ANGLE)));
  }
}
