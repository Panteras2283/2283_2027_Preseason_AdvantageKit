// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public boolean kickerConnected = false;
    public double kickerAppliedPercent = 0.0;
    public double kickerCurrentAmps = 0.0;

    public double rollerAppliedPercent = 0.0;
    public double rollerCurrentAmps = 0.0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setDutyCycle(double kickerPercent, double rollerPercent) {}
}
