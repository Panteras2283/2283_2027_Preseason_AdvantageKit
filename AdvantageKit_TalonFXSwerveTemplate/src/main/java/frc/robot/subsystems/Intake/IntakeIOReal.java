// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.Intake.IntakeConstants;

/** Add your docs here. */
public class IntakeIOReal implements IntakeIO {
    private final TalonFX pivotLeft = new TalonFX(IntakeConstants.PivotLeftID);
    private final TalonFX pivotRight = new TalonFX(IntakeConstants.PivotRightID);
    private final TalonFX feederLeft = new TalonFX(IntakeConstants.FeederLeftID);
    private final TalonFX feederRight = new TalonFX(IntakeConstants.FeederRightID);

    private final MotionMagicVoltage leftPivotRequest = new MotionMagicVoltage(0);
    private final MotionMagicVoltage rightPivotRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage feederVelRequest = new VelocityVoltage(0);
    private final DutyCycleOut feederDutyRequest = new DutyCycleOut(0);

    public IntakeIOReal(){
        configureFeeder();
        configureMotionMagic();
    }

    private void configureFeeder(){
        TalonFXConfiguration cfgF = new TalonFXConfiguration();
        cfgF.CurrentLimits.StatorCurrentLimit = 150.0;
        cfgF.CurrentLimits.StatorCurrentLimitEnable = true;
        cfgF.Slot0.kP = 0.33;
        cfgF.Slot0.kV = 0.12;

        feederRight.getConfigurator().apply(cfgF);
        feederLeft.getConfigurator().apply(cfgF);
        feederRight.setNeutralMode(NeutralModeValue.Coast);
        feederLeft.setNeutralMode(NeutralModeValue.Coast);
        feederLeft.setControl(new Follower(IntakeConstants.FeederRightID, MotorAlignmentValue.Opposed));
    }

    private void configureMotionMagic(){
        TalonFXConfiguration cfgMm = new TalonFXConfiguration();
        cfgMm.Slot0.kP = 20.0;
        cfgMm.Slot0.kI = 0.0;
        cfgMm.Slot0.kD = 0.0;
        cfgMm.Slot0.kV = 0.12;
        cfgMm.Slot0.kS = 0.2;
        cfgMm.MotionMagic.MotionMagicCruiseVelocity = 180;
        cfgMm.MotionMagic.MotionMagicAcceleration = 230;
        cfgMm.MotionMagic.MotionMagicJerk = 0;
        cfgMm.CurrentLimits.StatorCurrentLimit = 60.0;
        cfgMm.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotLeft.getConfigurator().apply(cfgMm);
        pivotRight.getConfigurator().apply(cfgMm);
        pivotLeft.setNeutralMode(NeutralModeValue.Brake);
        pivotRight.setNeutralMode(NeutralModeValue.Brake);
        pivotLeft.setPosition(0);
        pivotRight.setPosition(0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.leftPivotPositionRots = pivotLeft.getPosition().getValueAsDouble();
        inputs.rightPivotPositionRots = pivotRight.getPosition().getValueAsDouble();
        inputs.feederRightVelocityRps = feederRight.getVelocity().getValueAsDouble();
        inputs.feederLeftVelocityRps = feederLeft.getVelocity().getValueAsDouble();

        inputs.appliedVolts = new double[]{
            pivotLeft.getMotorVoltage().getValueAsDouble(),
            pivotRight.getMotorVoltage().getValueAsDouble(),
            feederLeft.getMotorVoltage().getValueAsDouble(),
            feederRight.getMotorVoltage().getValueAsDouble()
        };

        inputs.statorCurrentsAmps = new double[]{
            pivotLeft.getStatorCurrent().getValueAsDouble(),
            pivotRight.getStatorCurrent().getValueAsDouble(),
            feederLeft.getStatorCurrent().getValueAsDouble(),
            feederRight.getStatorCurrent().getValueAsDouble()
        };
    }
    @Override
    public void setPivotPositions(double leftPos, double rightPos){
        pivotLeft.setControl(leftPivotRequest.withPosition(leftPos));
        pivotRight.setControl(rightPivotRequest.withPosition(rightPos));
    }

    @Override
    public void setFeederVelocity(double velocity){
        feederRight.setControl(feederVelRequest.withVelocity(velocity));
    }

    @Override
    public void setFeederDutyCycle(double percent){
        feederRight.setControl(feederDutyRequest.withOutput(percent));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        NeutralModeValue mode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        pivotLeft.setNeutralMode(mode);
        pivotRight.setNeutralMode(mode);
    }
}
