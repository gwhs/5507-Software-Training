// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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

public class PivotSubsystem extends SubsystemBase {
  private final TalonFX motor;
  private final CANcoder encoder;

  private double goalPosition = 0;

  public final Trigger isAtPosition = new Trigger(() -> MathUtil.isNear(goalPosition, getPosition(), 10));
  public final Trigger isAtZero = new Trigger(() -> getCurrent() > 18 && getVelocity() < 1).debounce(1);

  private final Alert motorNotConnectedAlert = new Alert("Pivot Motor Not Connected", AlertType.kError);

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Current> motorStatorCurrent;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<AngularVelocity> motorVelocity;

  private final MotionMagicVoltage request = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem(CANBus canBus) {
    motor = new TalonFX(32, canBus);
    encoder = new CANcoder(33, canBus);

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

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.Feedback.RotorToSensorRatio = 16;
    config.Feedback.SensorToMechanismRatio = 1;

    motor.getConfigurator().apply(config);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0;
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    encoder.getConfigurator().apply(encoderConfig);

    motorVoltage = motor.getMotorVoltage();
    motorTemp = motor.getDeviceTemp();
    motorStatorCurrent = motor.getStatorCurrent();
    motorPosition = motor.getPosition();
    motorVelocity = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50, motorVoltage, motorTemp, motorStatorCurrent, motorPosition, motorVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorNotConnectedAlert.set(!motor.isConnected());

    BaseStatusSignal.refreshAll(motorVoltage, motorTemp, motorStatorCurrent, motorPosition, motorVelocity);

    DogLog.log("Pivot/Motor Voltage", motorVoltage.getValueAsDouble());
    DogLog.log("Pivot/Motor Temp", motorTemp.getValueAsDouble());
    DogLog.log("Pivot/Stator Current", motorStatorCurrent.getValueAsDouble());
    DogLog.log("Pivot/Motor Position", motorPosition.getValueAsDouble());
    DogLog.log("Pivot/Motor Velocity", motorVelocity.getValueAsDouble());

    DogLog.log("Pivot/isAtPosition", isAtPosition.getAsBoolean());
    DogLog.log("Pivot/isAtZero", isAtZero.getAsBoolean());
  }

  public Command runPosition (double rotation) {
    return this.runOnce(() -> {
      motor.setControl(request.withPosition(rotation));
      goalPosition = rotation;
    });
  }

  private Command runVoltage(double volt) {
    return this.runOnce(() -> {
      motor.setControl(voltageRequest.withOutput(volt).withIgnoreSoftwareLimits(true));
    });
  }

  private Command setPosition(double rotation) {
    return this.runOnce(() -> {
      motor.setPosition(rotation);
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
    return motorPosition.getValueAsDouble();
  }

  public double getCurrent() {
    return motorStatorCurrent.getValueAsDouble();
  }

  public double getVelocity() {
    return motorVelocity.getValueAsDouble();
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
    var encoderSim = encoder.getSimState();

    // set the supply voltage of the TalonFX
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    double motorVoltage = talonFXSim.getMotorVoltage();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage);
    motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    encoderSim.setRawPosition(motorSimModel.getAngularPosition().times(kGearRatio));
    encoderSim.setVelocity(motorSimModel.getAngularVelocity().times(kGearRatio));
  }
}
