// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

/** Add your docs here. */
public final class TurretConstants {
  public static final int TurretID = 25;

  public static final double kGearRatio = 38.8889;
  public static final double kSoftLimitForwardRot = 0.48;
  public static final double kSoftLimitBackwardsRot = -0.48;
  public static final double kAimToleranceDeg = 0.08;

  public static final double kP = 50;
  public static final double kI = 0.0;
  public static final double kD = 0.1;
  public static final double kV = 4.67;
  public static final double kS = 0.5;

  public static final double kCruiseVelocityRotPerSec = 2.5;
  public static final double kMaxAccelRotPerSec2 = 25.0;

  public static final double kStatorCurrentLimit = 60.0;

  private TurretConstants() {}
}
