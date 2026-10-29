// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

/** Add your docs here. */
public class KickerIOSim implements KickerIO {
  private double kickerPercent = 0.0;
  private double rollerPercent = 0.0;

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerConnected = true;
    inputs.kickerAppliedPercent = kickerPercent;
    inputs.kickerCurrentAmps = Math.abs(kickerPercent) * 20.0;

    inputs.rollerAppliedPercent = rollerPercent;
    inputs.rollerCurrentAmps = Math.abs(rollerPercent) * 20.0;
  }

  @Override
  public void setDutyCycle(double kickerPercent, double rollerPercent) {
    this.kickerPercent = kickerPercent;
    this.rollerPercent = rollerPercent;
  }
}
