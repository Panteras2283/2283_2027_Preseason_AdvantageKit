// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO{
    private final TalonFX flywheelLeft = new TalonFX(ShooterConstants.FlywheelLeftID);
    private final TalonFX flywheelRight = new TalonFX(ShooterConstants.FlywheelRightID);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public ShooterIOReal(){
        var flywheelRightConfig = new TalonFXConfiguration();
        flywheelRightConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kStatorCurrentLimit;
        flywheelRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelRightConfig.Slot0.kP = ShooterConstants.kP;
        flywheelRightConfig.Slot0.kI = ShooterConstants.kI;
        flywheelRightConfig.Slot0.kV = ShooterConstants.kV;
        flywheelRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelRight.getConfigurator().apply(flywheelRightConfig);
        flywheelRight.setNeutralMode(NeutralModeValue.Coast);

        var flywheelLeftConfig = new TalonFXConfiguration();
        flywheelLeftConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kStatorCurrentLimit;
        flywheelLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelLeftConfig.Slot0.kP = ShooterConstants.kP;
        flywheelLeftConfig.Slot0.kI = ShooterConstants.kI;
        flywheelLeftConfig.Slot0.kV = ShooterConstants.kV;
        flywheelLeft.getConfigurator().apply(flywheelLeftConfig);
        flywheelLeft.setNeutralMode(NeutralModeValue.Coast);
        flywheelLeft.setControl(new Follower(ShooterConstants.FlywheelRightID, MotorAlignmentValue.Opposed));
    }
}
