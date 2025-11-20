package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorConstants;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;

public class Auto_Coral extends SequentialCommandGroup {
  public Auto_Coral(
      RobotContainer robotContainer,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      EndEffectorSubsystem endEffector) {

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
          new Pose2d(Start_F.getPoint(0).position, Start_F.getIdealStartingState().rotation());

      addCommands(
          AutoBuilder.resetOdom(startingPose).onlyIf(() -> RobotBase.isSimulation()),
          /*
           * TODO: The rest of the autonomous routine command
           */
          AutoBuilder.followPath(Start_F)
              .deadlineFor(
                  Commands.sequence(Commands.waitSeconds(1.6 - 1.3)),
                  robotContainer.prepScoreCoral(
                      ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION)),
          endEffector.runVoltage(EndEffectorConstants.VOLTAGE_L4),
          AutoBuilder.followPath(F_CS)
              .deadlineFor(
                  elevator.runHeight(ElevatorConstants.INTAKE_METER),
                  arm.runAngle(ArmConstants.ARM_INTAKE_ANGLE)),
          endEffector.runVoltage(EndEffectorConstants.INTAKE_CORAL_VOLTAGE),
          AutoBuilder.followPath(CS_C)
              .deadlineFor(
                  Commands.sequence(
                      Commands.waitSeconds(2.0 - 1.3),
                      robotContainer.prepScoreCoral(
                          ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION))),
          endEffector.runVoltage(EndEffectorConstants.VOLTAGE_L4),
          AutoBuilder.followPath(C_CS)
              .deadlineFor(
                  elevator.runHeight(ElevatorConstants.INTAKE_METER),
                  arm.runAngle(ArmConstants.ARM_INTAKE_ANGLE)),
          endEffector.runVoltage(EndEffectorConstants.INTAKE_CORAL_VOLTAGE),
          AutoBuilder.followPath(CS_D)
              .deadlineFor(
                  Commands.sequence(
                      Commands.waitSeconds(2.1 - 1.3),
                      robotContainer.prepScoreCoral(
                          ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION))),
          endEffector.runVoltage(EndEffectorConstants.VOLTAGE_L4),
          AutoBuilder.followPath(D_CS)
              .deadlineFor(
                  elevator.runHeight(ElevatorConstants.INTAKE_METER),
                  arm.runAngle(ArmConstants.ARM_INTAKE_ANGLE)),
          endEffector.runVoltage(EndEffectorConstants.INTAKE_CORAL_VOLTAGE),
          AutoBuilder.followPath(CS_E)
              .deadlineFor(
                  Commands.sequence(
                      Commands.waitSeconds(2.3 - 1.3),
                      robotContainer.prepScoreCoral(
                          ElevatorConstants.L4_PREP_POSITION, ArmConstants.L4_PREP_POSITION))),
          endEffector.runVoltage(EndEffectorConstants.VOLTAGE_L4),
          AutoBuilder.followPath(E_CS)
              .deadlineFor(
                  elevator.runHeight(ElevatorConstants.INTAKE_METER),
                  arm.runAngle(ArmConstants.ARM_INTAKE_ANGLE)),
          endEffector.runVoltage(EndEffectorConstants.INTAKE_CORAL_VOLTAGE));
    } catch (Exception e) {
      DriverStation.reportError("Path Not Found: " + e.getMessage(), e.getStackTrace());
    }
  }
}
