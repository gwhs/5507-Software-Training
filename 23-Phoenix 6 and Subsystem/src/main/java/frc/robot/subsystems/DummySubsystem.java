package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  private Orchestra orch = new Orchestra();

  public DummySubsystem() {
    orch.loadMusic("output.chrp");
    orch.addInstrument(motor);
    SmartDashboard.putData("spin -1V", runVoltage(-1));
    SmartDashboard.putData("spin 0V", runVoltage(0));
    SmartDashboard.putData("spin 1V", runVoltage(1));
    SmartDashboard.putData("evil >:)", evilMeme());
    motorConf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConf.Audio.BeepOnConfig = true;
    motorConf.Audio.BeepOnBoot = true;
    motorConf.Audio.AllowMusicDurDisable = false;
    motorConf.CurrentLimits.StatorCurrentLimit = 3;
    motorConf.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConf.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConf.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    motorConf.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConf.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;
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

  public Command evilMeme() {
    return this.runOnce(() -> orch.play());
  }

  @Override
  public void periodic() {}
}
