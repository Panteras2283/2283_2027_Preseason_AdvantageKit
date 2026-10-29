// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.kicker.KickerIO.KickerIOInputs;

/** Add your docs here. */
public class KickerIOReal implements KickerIO {
  private final TalonFX kickerMotor = new TalonFX(KickerConstants.KickermotorID);
  private final SparkMax rollerMotor = new SparkMax(KickerConstants.RollerID, MotorType.kBrushless);

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerConnected = kickerMotor.isConnected();
    inputs.kickerAppliedPercent = kickerMotor.getDutyCycle().getValueAsDouble();
    inputs.kickerCurrentAmps = kickerMotor.getStatorCurrent().getValueAsDouble();

    inputs.rollerAppliedPercent = rollerMotor.getAppliedOutput();
    inputs.rollerCurrentAmps = rollerMotor.getOutputCurrent();
  }

  @Override
  public void setDutyCycle(double kickerPercent, double rollerPercent) {
    kickerMotor.set(-kickerPercent);
    rollerMotor.set(rollerPercent);
  }
}
