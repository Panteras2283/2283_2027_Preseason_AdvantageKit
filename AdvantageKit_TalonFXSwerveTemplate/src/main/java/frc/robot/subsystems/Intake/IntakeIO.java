// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean pivotConnected = false;

    public double leftPivotPositionRots = 0.0;
    public double rightPivotPositionRots = 0.0;

    public boolean feederConnected = false;
    public double feederRightVelocityRps = 0.0;
    public double feederLeftVelocityRps = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPivotPositions(double leftPos, double rightPos) {}

  public default void setFeederVelocity(double velocity) {}

  public default void setFeederDutyCycle(double percent) {}

  public default void setBrakeMode(boolean enable) {}
}
