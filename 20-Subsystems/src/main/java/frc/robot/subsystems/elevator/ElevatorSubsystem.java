// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX motor1;
  private final TalonFX motor2;

  private double goalPosition = 0;

  public final Trigger isAtPosition = new Trigger(() -> MathUtil.isNear(goalPosition, getPosition(), 1));
  public final Trigger isAtZero = new Trigger(() -> getCurrent() > 18 && getVelocity() < 1).debounce(1);

  private final Alert motor1NotConnectedAlert = new Alert("Elevator Motor 1 Not Connected", AlertType.kError);
  private final Alert motor2NotConnectedAlert = new Alert("Elevator Motor 2 Not Connected", AlertType.kError);

  private final StatusSignal<Voltage> motor1Voltage;
  private final StatusSignal<Temperature> motor1Temp;
  private final StatusSignal<Current> motor1StatorCurrent;
  private final StatusSignal<Angle> motor1Position;
  private final StatusSignal<AngularVelocity> motor1Velocity;

  private final StatusSignal<Voltage> motor2Voltage;
  private final StatusSignal<Temperature> motor2Temp;
  private final StatusSignal<Current> motor2StatorCurrent;
  private final StatusSignal<Angle> motor2Position;
  private final StatusSignal<AngularVelocity> motor2Velocity;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final Follower followerRequest = new Follower(11, MotorAlignmentValue.Opposed);

  /** Creates a new PivotSubsystem. */
  public ElevatorSubsystem(CANBus canBus) {
    motor1 = new TalonFX(11, canBus);
    motor2 = new TalonFX(12, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40;

    config.Slot0.kS = 0;
    config.Slot0.kV = 0.1125;
    config.Slot0.kP = 2;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; 
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    config.MotionMagic.MotionMagicAcceleration = 0.1;
    config.MotionMagic.MotionMagicCruiseVelocity = 0.5;

    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    motor1Voltage = motor1.getMotorVoltage();
    motor1Temp = motor1.getDeviceTemp();
    motor1StatorCurrent = motor1.getStatorCurrent();
    motor1Position = motor1.getPosition();
    motor1Velocity = motor1.getVelocity();

    motor2Voltage = motor2.getMotorVoltage();
    motor2Temp = motor2.getDeviceTemp();
    motor2StatorCurrent = motor2.getStatorCurrent();
    motor2Position = motor2.getPosition();
    motor2Velocity = motor2.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50, motor1Temp, motor1StatorCurrent, motor1Position, motor1Velocity, motor2Temp, motor2StatorCurrent, motor2Position, motor2Velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(250, motor1Voltage, motor2Voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motor1NotConnectedAlert.set(!motor1.isConnected());
    motor2NotConnectedAlert.set(!motor2.isConnected());

    BaseStatusSignal.refreshAll(motor1Voltage, motor1Temp, motor1StatorCurrent, motor1Position, motor1Velocity, motor2Voltage, motor2Temp, motor2StatorCurrent, motor2Position, motor2Velocity);

    DogLog.log("Elevator/Motor 1 Voltage", motor1Voltage.getValueAsDouble());
    DogLog.log("Elevator/Motor 1 Temp", motor1Temp.getValueAsDouble());
    DogLog.log("Elevator/Motor 1 Stator Current", motor1StatorCurrent.getValueAsDouble());
    DogLog.log("Elevator/Motor 1 Position", motor1Position.getValueAsDouble());
    DogLog.log("Elevator/Motor 1 Velocity", motor1Velocity.getValueAsDouble());
    DogLog.log("Elevator/Motor 1 Is Connected", motor1.isConnected());

    DogLog.log("Elevator/Motor 2 Voltage", motor2Voltage.getValueAsDouble());
    DogLog.log("Elevator/Motor 2 Temp", motor2Temp.getValueAsDouble());
    DogLog.log("Elevator/Motor 2 Stator Current", motor2StatorCurrent.getValueAsDouble());
    DogLog.log("Elevator/Motor 2 Position", motor2Position.getValueAsDouble());
    DogLog.log("Elevator/Motor 2 Velocity", motor2Velocity.getValueAsDouble());
    DogLog.log("Elevator/Motor 2 Is Connected", motor2.isConnected());

    DogLog.log("Elevator/isAtPosition", isAtPosition.getAsBoolean());
    DogLog.log("Elevator/isAtZero", isAtZero.getAsBoolean());
  }

  public Command runPosition (double rotation) {
    return this.runOnce(() -> {
      motor1.setControl(request.withPosition(rotation));
      motor2.setControl(followerRequest);
      goalPosition = rotation;
    });
  }

  private Command runVoltage(double volt) {
    return this.runOnce(() -> {
      motor1.setControl(voltageRequest.withOutput(volt).withIgnoreSoftwareLimits(true));
      motor2.setControl(followerRequest);
    });
  }

  private Command setPosition(double rotation) {
    return this.runOnce(() -> {
      motor1.setPosition(rotation);
      motor2.setPosition(rotation);
    });
  }

  public Command homing() {
    return Commands.sequence(
      runVoltage(-1),
      Commands.waitUntil(isAtZero),
      runVoltage(0),
      setPosition(0)
    );
  }

  public double getPosition() {
    return motor1Position.getValueAsDouble();
  }

  public double getCurrent() {
    return motor1StatorCurrent.getValueAsDouble();
  }

  public double getVelocity() {
    return motor1Velocity.getValueAsDouble();
  }

    /* Simulation */
  private static final double kGearRatio = 1.0;
  private final DCMotorSim motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.001, kGearRatio),
          DCMotor.getKrakenX60Foc(2));

  @Override
  public void simulationPeriodic() {
    var talonFXSim = motor1.getSimState();
    var talonFXSim2 = motor2.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    talonFXSim2.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    double motorVoltage = talonFXSim.getMotorVoltage();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage);
    motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(kGearRatio));
    talonFXSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(kGearRatio));

    talonFXSim2.setRawRotorPosition(motorSimModel.getAngularPosition().times(kGearRatio));
    talonFXSim2.setRotorVelocity(motorSimModel.getAngularVelocity().times(kGearRatio));
  }
}
