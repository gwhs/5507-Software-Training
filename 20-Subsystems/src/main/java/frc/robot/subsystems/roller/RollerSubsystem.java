// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  private final TalonFX motor;
  private final TalonFX motor2;

  private final Alert motorNotConnectedAlert = new Alert("Roller Motor 1 Not Connected", AlertType.kError);
  private final Alert motor2NotConnectedAlert = new Alert("Roller Motor 2 Not Connected", AlertType.kError);

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Current> motorStatorCurrent;

  private final StatusSignal<Voltage> motor2Voltage;
  private final StatusSignal<Temperature> motor2Temp;
  private final StatusSignal<Current> motor2StatorCurrent;

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final Follower followerRequest = new Follower(40, MotorAlignmentValue.Opposed);

  /** Creates a new ShooterSubsystem. */
  public RollerSubsystem(CANBus canBus) {
    motor = new TalonFX(40, canBus);
    motor2 = new TalonFX(41, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;

    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    motorVoltage = motor.getMotorVoltage();
    motorTemp = motor.getDeviceTemp();
    motorStatorCurrent = motor.getStatorCurrent();

    motor2Voltage = motor2.getMotorVoltage();
    motor2Temp = motor2.getDeviceTemp();
    motor2StatorCurrent = motor2.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50, motorTemp, motorStatorCurrent, motor2Temp, motor2StatorCurrent);

    BaseStatusSignal.setUpdateFrequencyForAll(250, motorVoltage, motor2Voltage, motor.getTorqueCurrent(), motor2.getTorqueCurrent());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorNotConnectedAlert.set(!motor.isConnected());
    motor2NotConnectedAlert.set(!motor2.isConnected());

    BaseStatusSignal.refreshAll(motorVoltage, motorTemp, motorStatorCurrent, motor2Voltage, motor2Temp, motor2StatorCurrent);

    DogLog.log("Roller/Motor 1 Voltage", motorVoltage.getValueAsDouble());
    DogLog.log("Roller/Motor 1 Device Temp", motorTemp.getValueAsDouble());
    DogLog.log("Roller/Motor 1 Stator Current", motorStatorCurrent.getValueAsDouble());

    DogLog.log("Roller/Motor 2 Voltage", motor2Voltage.getValueAsDouble());
    DogLog.log("Roller/Motor 2 Device Temp", motor2Temp.getValueAsDouble());
    DogLog.log("Roller/Motor 2 Stator Current", motor2StatorCurrent.getValueAsDouble());
  }

  public Command runVoltage(double volt) {
    return this.runOnce(() -> {
      motor.setVoltage(volt);
      motor2.setControl(followerRequest);
    });
  }

  public Command runCurrent(double amp, double dutyCycle) {
    return this.runOnce(() -> {
      motor.setControl(torqueCurrentRequest.withOutput(amp).withMaxAbsDutyCycle(dutyCycle));
      motor2.setControl(followerRequest);
    });
  }

  /* Simulation */
  private static final double kGearRatio = 1.0;
  private final DCMotorSim motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio),
          DCMotor.getKrakenX60Foc(1));

  @Override
  public void simulationPeriodic() {
    var talonFXSim = motor.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = talonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(kGearRatio));
    talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(kGearRatio));
  }

}
