package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummySubsystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(1, "rio");
  private TalonFXConfiguration motorConf = new TalonFXConfiguration();
  private TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);
  private VelocityVoltage velocityRequest = new VelocityVoltage(0);

  @SuppressWarnings("resource")
  public DummySubsystem() {
    SmartDashboard.putData("spin -1V", runVoltage(-1));
    SmartDashboard.putData("spin 0V", runVoltage(0));
    SmartDashboard.putData("spin 1V", runVoltage(1));
    SmartDashboard.putData("put 1 amp", runCurrent(3, 1));
    SmartDashboard.putData("put 10 amp", runCurrent(10, 1));
    SmartDashboard.putData("put 10,0.2 amp", runCurrent(10, 0.2));
    SmartDashboard.putData("put -10 amp", runCurrent(-10, 0.2));
    SmartDashboard.putData("put 0.2 vel", runVelocity(0.2));
    SmartDashboard.putData("put 1 vel", runVelocity(1));
    SmartDashboard.putData("put 5 vel", runVelocity(5));
    SmartDashboard.putData("put -0.2 vel", runVelocity(-0.2));

    motorConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConf.Audio.BeepOnConfig = true;
    motorConf.Audio.BeepOnBoot = true;
    motorConf.Audio.AllowMusicDurDisable = false;
    motorConf.CurrentLimits.StatorCurrentLimit = 15;
    motorConf.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // motorConf.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // motorConf.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    // motorConf.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // motorConf.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;
    motorConf.Slot0.kS = 0;
    motorConf.Slot0.kG = 0;
    motorConf.Slot0.kA = 0;
    motorConf.Slot0.kV = 0.1125;
    motorConf.Slot0.kP = 1.2;
    motorConf.Slot0.kI = 0;
    motorConf.Slot0.kD = 0;
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = motor.getConfigurator().apply(motorConf);
      if (status.isOK()) break;
    }
    if (status.isError()) {
      new Alert("ERROR: couldn't configure motor: " + status.toString(), AlertType.kError)
          .set(true);
    }
  }

  public Command runVoltage(double volts) {
    return this.runOnce(() -> motor.setVoltage(volts));
  }

  public Command runCurrent(double amps, double dutyCycle) {
    return this.runOnce(
        () -> {
          motor.setControl(currentRequest.withOutput(amps).withMaxAbsDutyCycle(dutyCycle));
        });
  }

  public Command runVelocity(double vel) {
    return this.runOnce(
        () -> {
          motor.setControl(velocityRequest.withVelocity(vel));
        });
  }

  @Override
  public void periodic() {}
}
