// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

  public IntakeIOReal() {
    configureFeeder();
    configureMotionMagic();
  }

  private void configureFeeder() {
    TalonFXConfiguration cfgF = new TalonFXConfiguration();
    cfgF.CurrentLimits.StatorCurrentLimit = IntakeConstants.kFeederStatorCurrentLimit;
    cfgF.CurrentLimits.StatorCurrentLimitEnable = true;
    cfgF.Slot0.kP = IntakeConstants.kFeederP;
    cfgF.Slot0.kV = IntakeConstants.kFeederV;

    feederRight.getConfigurator().apply(cfgF);
    feederLeft.getConfigurator().apply(cfgF);
    feederRight.setNeutralMode(NeutralModeValue.Coast);
    feederLeft.setNeutralMode(NeutralModeValue.Coast);
    feederLeft.setControl(new Follower(IntakeConstants.FeederRightID, MotorAlignmentValue.Opposed));
  }

  private void configureMotionMagic() {
    TalonFXConfiguration cfgMm = new TalonFXConfiguration();
    cfgMm.Slot0.kP = IntakeConstants.kPivotP;
    cfgMm.Slot0.kI = IntakeConstants.kPivotI;
    cfgMm.Slot0.kD = IntakeConstants.kPivotD;
    cfgMm.Slot0.kV = IntakeConstants.kPivotV;
    cfgMm.Slot0.kS = IntakeConstants.kPivotS;
    cfgMm.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kPivotCruiseVelocityRotPerSec;
    cfgMm.MotionMagic.MotionMagicAcceleration = IntakeConstants.kPivotMaxAccelRotPerSec2;
    cfgMm.CurrentLimits.StatorCurrentLimit = IntakeConstants.kPivotStatorCurrentLimit;
    cfgMm.CurrentLimits.StatorCurrentLimitEnable = true;

    pivotLeft.getConfigurator().apply(cfgMm);
    pivotRight.getConfigurator().apply(cfgMm);
    pivotLeft.setNeutralMode(NeutralModeValue.Brake);
    pivotRight.setNeutralMode(NeutralModeValue.Brake);
    pivotLeft.setPosition(0);
    pivotRight.setPosition(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotConnected = pivotLeft.isConnected() && pivotRight.isConnected();
    inputs.leftPivotPositionRots = pivotLeft.getPosition().getValueAsDouble();
    inputs.rightPivotPositionRots = pivotRight.getPosition().getValueAsDouble();

    inputs.feederConnected = feederRight.isConnected();
    inputs.feederRightVelocityRps = feederRight.getVelocity().getValueAsDouble();
    inputs.feederLeftVelocityRps = feederLeft.getVelocity().getValueAsDouble();
    inputs.feederCurrentAmps = feederRight.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setPivotPositions(double leftPos, double rightPos) {
    pivotLeft.setControl(leftPivotRequest.withPosition(leftPos));
    pivotRight.setControl(rightPivotRequest.withPosition(rightPos));
  }

  @Override
  public void setFeederVelocity(double velocity) {
    feederRight.setControl(feederVelRequest.withVelocity(velocity));
  }

  @Override
  public void setFeederDutyCycle(double percent) {
    feederRight.setControl(feederDutyRequest.withOutput(percent));
  }

  @Override
  public void setBrakeMode(boolean enable) {
    NeutralModeValue mode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    pivotLeft.setNeutralMode(mode);
    pivotRight.setNeutralMode(mode);
  }
}
