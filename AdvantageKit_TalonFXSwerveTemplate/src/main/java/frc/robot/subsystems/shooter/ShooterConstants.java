// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public final class ShooterConstants {
    public static final int FlywheelRightID = 27;
    public static final int FlywheelLeftID = 29;

    public static final double kStatorCurrentLimit = 150.0;

    public static final double kP = 0.45;
    public static final double kI = 0.0;
    public static final double kV = 0.12;

    public static final double kOvershootCutoffRPM = 100.0;
    public static final double kReadyToFireTolerancePercent = 0.15;
    public static final double kMinRPMToBeReady = 100.0;

    private ShooterConstants() {}
}
