// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
@AutoLog
public static class IntakeIOInputs {
    public boolean connected = false;

    public double leftPivotPositionRots = 0.0;
    public double rightPivotPositionRots = 0.0;

    public double feederRightVelocityRps = 0.0;
    public double feederLeftVelocityRps = 0.0;

    public double[] appliedVolts = new double[] {0.0, 0.0, 0.0, 0.0};
    public double[] statorCurrentsAmps = new double[] {0.0, 0.0, 0.0, 0.0};

}
    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setPivotPositions(double leftPos, double rightPos) {}
    public default void setFeederVelocity(double velocity){}
    public default void setFeederDutyCycle(double percent) {}
    public default void setBrakeMode(boolean enable) {}
} 
