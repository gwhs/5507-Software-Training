// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ser.std.StaticListSerializerBase;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
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

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final int TalonFX = 21;
  //private final TalonFX motor1;
  private final TalonFX motor; 

  private double goalVelocity = 0;

  private final VelocityVoltage request = new VelocityVoltage(0);


  public final Trigger isAtVelocity = new Trigger(() -> MathUtil.isNear (goalVelocity, getVelocity(), 10));
  private final Alert motorNotConnectedAlert = new Alert("Shooter Motor Not Connected", AlertType.kError); //run on wednesday slide 83
  
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;

  public ShooterSubsystem(CANBus canBus) {
    motor = new TalonFX(21, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; //run on wednesday slide 77

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 10.0; //run on wednesday slide 67

    config.Slot0.kS = 0;
    config.Slot0.kV = 0;
    config.Slot0.kP = 0; //workshop 23, slide 26

    motor.getConfigurator().apply(config); //run on wednesday pg 57
    //motor1 = new TalonFX(22, canBus);
    motorVoltage = motor.getMotorVoltage();
    motorTemp = motor.getDeviceTemp();
    motorStatorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50, motorVoltage, motorTemp, motorStatorCurrent, motorVelocity);

  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
        //motor.setVoltage(1);
        //motor1.setVoltage(1);
        motorNotConnectedAlert.set(!motor.isConnected()); //slide 83
        
        BaseStatusSignal.refreshAll(motorVoltage, motorTemp, motorStatorCurrent, motorVelocity);

        DogLog.log("Shooter/Motor Voltage", motorVoltage.getValueAsDouble());
        DogLog.log("Shooter/Motor Temperature", motorTemp.getValueAsDouble());
        DogLog.log("Shooter/Motor Stator Current", motorStatorCurrent.getValueAsDouble());
        DogLog.log("Shooter/Motor Velocity", motorVelocity.getValueAsDouble());
        DogLog.log("Shooter/Velocity Goal", goalVelocity);
        DogLog.log("Shooter/isAtVelocity", isAtVelocity.getAsBoolean());

        BaseStatusSignal.setUpdateFrequencyForAll(50, motorVoltage, motorTemp, motorStatorCurrent, motorVelocity);

      }
  public Command startShooter() {
    return this.runOnce(() -> {
      motor.setVoltage(1);
    });
  }

  public Command stopShooter() {
    return this.runOnce(() -> {
      motor.setVoltage((0));
    });
  }
  public Command runVoltage(double volts) {
    return this.runOnce(() -> {
      motor.setVoltage(volts);
    });
  }

  public double getVelocity() {
    return motorVelocity.getValueAsDouble();
    }

  public Command runVelocity(double rps) {
    return this.runOnce(() -> {
      motor.setControl(request.withVelocity(rps));
      goalVelocity = rps;
    });
  }
}
