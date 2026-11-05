// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */
  private final TalonFX motor;

  private double goalPosition = 0;

  private final Trigger isAtPosition = new Trigger(() -> MathUtil.isNear(goalPosition, getPosition(), 10));

  private final Alert motorNotConnectedAlert = new Alert ("Pivot Motor Not Connected", AlertType.kError);

  private final StatusSignal <Voltage> motorVoltage;
  private final StatusSignal <Temperature> motorTemp;
  private final StatusSignal <Current> motorStatorCurrent;
  private final StatusSignal <Angle> motorPosition;
  private final StatusSignal <AngularVelocity> motorVelocity;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);

  private final PositionVoltage request = new PositionVoltage(0);

  public PivotSubsystem(CANBus canBus) {
    motor = new TalonFX (32, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 20;

    config.Slot0.kS = 0;
    config.Slot0.kV = 0.1125;
    config.Slot0.kP = 2;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    config.MotionMagic.MotionMagicAcceleration = 0.1;
    config.MotionMagic.MotionMagicCruiseVelocity = 0.5;

    motor.getConfigurator().apply(config);

    motorVoltage = motor.getMotorVoltage();
    motorTemp =  motor.getDeviceTemp();
    motorStatorCurrent = motor.getStatorCurrent();
    motorPosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    
    BaseStatusSignal.setUpdateFrequencyForAll(50, motorVoltage, motorTemp, motorStatorCurrent, motorPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
            motorNotConnectedAlert.set(!motor.isConnected()); 
        
        BaseStatusSignal.refreshAll(motorVoltage, motorTemp, motorStatorCurrent, motorPosition, motorVelocity);

        DogLog.log("Pivot/Motor Voltage", motorVoltage.getValueAsDouble());
        DogLog.log("Pivot/Motor Temperature", motorTemp.getValueAsDouble());
        DogLog.log("Pivot/Motor Stator Current", motorStatorCurrent.getValueAsDouble());
        DogLog.log("Pivot/Motor Position", motorPosition.getValueAsDouble());
        DogLog.log("Pivot/Motor Velocity", isAtPosition.getAsBoolean());

        DogLog.log("Pivot/isAtPosition", isAtPosition.getAsBoolean());
  }

  public Command runPosition(double rotation) {
    return this.runOnce(() -> {
      motor.setControl(request.withPosition(rotation));
      goalPosition = rotation;
      });
    }

  public double getPosition() {
    return motorPosition.getValueAsDouble();
  }
}
