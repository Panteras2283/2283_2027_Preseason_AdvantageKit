// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface SpindexerIO {
    @AutoLog
    public static class SpindexerIOInputs {
        public boolean connected = false;
        public double velocityRPM = 0.0;
        public double appliedOutput = 0.0;
        public double currentAmps = 0.0;
    }

    public default void updateInputs(SpindexerIOInputs inputs) {}

    public default void setVelocityRPM(double rpm) {}

    public default void stop() {}
}
