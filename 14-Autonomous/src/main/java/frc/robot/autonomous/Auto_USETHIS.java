package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorConstants;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;

public class Auto_USETHIS extends SequentialCommandGroup {
  public Auto_USETHIS(
      ArmSubsystem arm, ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {

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
          new Pose2d(S_F.getPoint(0).position, S_F.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          AutoBuilder.followPath(S_F),
          Commands.sequence(
              elevator.runHeight(ElevatorConstants.L4_PREP_POSITION),
              arm.runAngle(ArmConstants.L4_PREP_POSITION),
              endEffector.runVoltage(EndEffectorConstants.VOLTAGE_L4)),
          Commands.waitSeconds(0.05),
          Commands.sequence(
              elevator.runHeight(ElevatorConstants.STOW_METER),
              arm.runAngle(ArmConstants.ARM_STOW_ANGLE),
              endEffector.runVoltage(0)),
          AutoBuilder.followPath(F_CS),
          Commands.parallel(
              arm.runAngle(ArmConstants.ARM_INTAKE_ANGLE),
              elevator.runHeight(ElevatorConstants.INTAKE_METER)),
          AutoBuilder.followPath(CS_C),
          Commands.sequence(
              elevator.runHeight(ElevatorConstants.L4_PREP_POSITION),
              arm.runAngle(ArmConstants.L4_PREP_POSITION),
              endEffector.runVoltage(EndEffectorConstants.VOLTAGE_L4)),
          Commands.waitSeconds(0.05),
          Commands.sequence(
              elevator.runHeight(ElevatorConstants.STOW_METER),
              arm.runAngle(ArmConstants.ARM_STOW_ANGLE),
              endEffector.runVoltage(0)),
          AutoBuilder.followPath(C_CS),
          Commands.parallel(
              arm.runAngle(ArmConstants.ARM_INTAKE_ANGLE),
              elevator.runHeight(ElevatorConstants.INTAKE_METER))
          /*
           * TODO: The rest of the autonomous routine command
           */

          );

    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
