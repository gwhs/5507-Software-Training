package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.Supplier;

public class AlignToPose extends Command {

  Supplier<Pose2d> targetPose;

  private SwerveSubsystem drivetrain;

  public Constraints constraints = new TrapezoidProfile.Constraints(3, 2);
  public ProfiledPIDController PID_X = new ProfiledPIDController(3.0, 0, 0, constraints);
  public ProfiledPIDController PID_Y = new ProfiledPIDController(3.0, 0, 0, constraints);
  public PIDController PID_Rotation = new PIDController(0.05, 0, 0);

  private double maxSpeed = 4.5;
  private double maxAngularRate = 1.0 * Math.PI;

  public static final double PID_MAX = 0.44;
  public static final double PID_ROTATION_MAX = 0.70;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.05)
          .withRotationalDeadband(maxAngularRate * 0.05)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public AlignToPose(SwerveSubsystem drivetrain, Supplier<Pose2d> targetPose) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
  }

  /**
   * @return if it is at pose true if not false
   */
  public boolean isAtTargetPose() {
    boolean isAtX = PID_X.atSetpoint();
    boolean isAtY = PID_Y.atSetpoint();
    boolean isAtRotation = PID_Rotation.atSetpoint();

    if (isAtX && isAtY && isAtRotation) {
      return true;
    }
    return false;
  }

  @Override
  public void initialize() {
    goToPoseWithPID(targetPose.get());
    DogLog.log("Align/Target Pose", targetPose.get());
  }

  @Override
  public void execute() {
    Pose2d currPose;
    currPose = drivetrain.getState().Pose;
    double currX = currPose.getX();
    double currY = currPose.getY();
    Double currRotation = currPose.getRotation().getDegrees();

    double PIDXOutput = MathUtil.clamp(PID_X.calculate(currX), -PID_MAX, PID_MAX);
    double xVelocity = -PIDXOutput;
    DogLog.log("Align/PIDXOutput", PIDXOutput);

    double PIDYOutput = MathUtil.clamp(PID_Y.calculate(currY), -PID_MAX, PID_MAX);
    double yVelocity = -PIDYOutput;
    DogLog.log("Align/PIDYoutput", PIDYOutput);

    double PIDRotationOutput =
        MathUtil.clamp(PID_Rotation.calculate(currRotation), -PID_ROTATION_MAX, PID_ROTATION_MAX);
    double angularVelocity = PIDRotationOutput;
    DogLog.log("Align/PIDRotationoutput", PIDRotationOutput);

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      xVelocity = -xVelocity * maxSpeed;
      yVelocity = -yVelocity * maxSpeed;
      angularVelocity = angularVelocity * maxAngularRate;
    } else {
      xVelocity = xVelocity * maxSpeed;
      yVelocity = yVelocity * maxSpeed;
      angularVelocity = angularVelocity * maxAngularRate;
    }

    angularVelocity = MathUtil.clamp(angularVelocity, -maxAngularRate, maxAngularRate);
    DogLog.log("Align/xVelocity", xVelocity);
    DogLog.log("Align/yVelocity", yVelocity);
    DogLog.log("Align/angularVelocity", angularVelocity);
    drivetrain.setControl(
        drive
            .withVelocityX(xVelocity) // Drive forward with negative Y (forward)
            .withVelocityY(yVelocity) // Drive left with negative X (left)
            .withRotationalRate(angularVelocity)); // Drive counterclockwise with negative X (left)
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityX(0.00).withVelocityY(0.00).withRotationalRate(0.00));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void goToPoseWithPID(Pose2d targetPose) {
    ChassisSpeeds currentSpeed =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());

    double predicted_X =
        (targetPose.getX() - drivetrain.getState().Pose.getX()) * 0.3
            + drivetrain.getState().Pose.getX();
    double predicted_Y =
        (targetPose.getY() - drivetrain.getState().Pose.getY()) * 0.3
            + drivetrain.getState().Pose.getY();

    PID_X.reset(predicted_X, currentSpeed.vxMetersPerSecond * 0.4);
    PID_Y.reset(predicted_Y, currentSpeed.vyMetersPerSecond * 0.4);
    PID_X.setGoal(targetPose.getX());
    PID_Y.setGoal(targetPose.getY());
    PID_Rotation.setSetpoint(targetPose.getRotation().getDegrees());
  }
}
