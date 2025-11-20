package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummySubsystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(1, "rio");

  public DummySubsystem() {
    SmartDashboard.putData("Run motor at 1V", runMotor(1));
    SmartDashboard.putData("Run motor at 0V", runMotor(0));
    SmartDashboard.putData("Run motor at -1V", runMotor(-1));
  }

  public Command runMotor(double voltage) {
    return this.runOnce(
        () -> {
          motor.setVoltage(voltage);
        });
  }

  @Override
  public void periodic() {}
}
