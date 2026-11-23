// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean turretConnected = false;
    public double positionRotations = 0.0;
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setPosition(double positionRotations) {}

  public default void setOpenLoop(double volts) {}

  public default void resetPosition() {}

  public default void stop() {}
}
