// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntakePivot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakePivotSubsystem extends SubsystemBase {
  private final CANBus canBus = new CANBus("CAN_Network");
  private final TalonFX motor = new TalonFX(GroundIntakePivotConstants.MOTOR_ID, canBus);
  private final CANcoder encoder = new CANcoder(GroundIntakePivotConstants.ENCODER_ID, canBus);

  private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);

  /** Creates a new GroundIntakePivotSubsystem. */
  public GroundIntakePivotSubsystem() {
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 10;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Feedback.FeedbackRotorOffset = 0;
    talonFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    talonFXConfig.Feedback.FeedbackRemoteSensorID = GroundIntakePivotConstants.ENCODER_ID;
    talonFXConfig.Feedback.SensorToMechanismRatio = 36.0 / 18.0;
    talonFXConfig.Feedback.RotorToSensorRatio = 42.0 / 12.0 * 42.0 / 38.0 * 62.0 / 18.0;

    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        GroundIntakePivotConstants.MAX_ROTATION;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        GroundIntakePivotConstants.MIN_ROTATION;

    talonFXConfig.MotionMagic.MotionMagicAcceleration = GroundIntakePivotConstants.MAX_ACCELERATION;
    talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = GroundIntakePivotConstants.MAX_VELOCITY;

    talonFXConfig.Slot0.kP = 33;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    talonFXConfig.Slot0.kS = 0.125;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.1125 * 26.6;

    motor.getConfigurator().apply(talonFXConfig);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoderConfig.MagnetSensor.MagnetOffset = 1;

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    encoder.getConfigurator().apply(encoderConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command deploy() {
    return Commands.sequence(
        this.runOnce(
            () -> {
              motor.setControl(request.withPosition(GroundIntakePivotConstants.DEPLOY_ROTATION));
            }),
        Commands.waitSeconds(2));
  }
}
