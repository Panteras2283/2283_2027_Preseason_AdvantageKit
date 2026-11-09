// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

/** Add your docs here. */
public class SpindexerConstants {
    public static final int SpindexerMotorID = 28;

    public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 12.0/380.0;

    public static final double kSpinClockwiseRPM = 4500;
    public static final double kSpinCounterclockwiseRPM = -1000;
    public static final double kUnjamRPM = -6000;

    public static final double kJamCurrentAmps = 200.0;
    public static final double kJamMaxRPM = 20.0;

    private SpindexerConstants() {}
}
