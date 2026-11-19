// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;


/** Add your docs here. */
public class RollerSubsystem extends SubsystemBase{
    private final TalonFx motor;
    private final Alert motorNotConnectedAlert = new Alert ("Roller Motor Not Connected", AlertType.kError);

    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Temperature> motorTemp;
    private final StatusSignal<Current> motorStatorCurrent;
    
    public RollerSubsystem(CANBus canBus) {
        motor = new TalonFX(40, canBus);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        

    }

}
