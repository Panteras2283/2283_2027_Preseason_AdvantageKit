// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class TurretIOReal implements TurretIO {
  private final TalonFX motor = new TalonFX(TurretConstants.TurretID);

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public TurretIOReal() {
    var config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = TurretConstants.kGearRatio;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.kSoftLimitForwardRot;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.kSoftLimitBackwardsRot;

    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kCruiseVelocityRotPerSec;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.kMaxAccelRotPerSec2;

    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kI = TurretConstants.kI;
    config.Slot0.kD = TurretConstants.kD;
    config.Slot0.kV = TurretConstants.kV;
    config.Slot0.kS = TurretConstants.kS;

    config.CurrentLimits.StatorCurrentLimit = TurretConstants.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Coast);
    motor.setPosition(0);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretConnected = motor.isConnected();
    inputs.positionRotations = motor.getPosition().getValueAsDouble();
    inputs.velocityRotPerSec = motor.getVelocity().getValueAsDouble();
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRotations) {
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  @Override
  public void setOpenLoop(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void resetPosition() {
    motor.setPosition(0);
  }

  @Override
  public void stop() {
    motor.setControl(new NeutralOut());
  }
}
