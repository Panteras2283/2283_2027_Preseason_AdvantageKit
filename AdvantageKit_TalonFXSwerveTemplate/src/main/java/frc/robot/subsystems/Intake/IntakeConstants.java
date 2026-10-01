// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeConstants {
  public static final int PivotLeftID = 23;
  public static final int PivotRightID = 30;
  public static final int FeederRightID = 31;
  public static final int FeederLeftID = 33;

  public static final double LeftFeedPos = -0.5;
  public static final double RightFeedPos = -0;
  public static final double LeftUpPos = 16;
  public static final double RightUpPos = -16;
  public static final double LeftShakeUpPos = 7.5;
  public static final double RightShakeUpPos = -7.5;
  public static final double LeftShakeDownPos = 2;
  public static final double RightShakeDownPos = -2;

  public static final double kFeederVelocityRotPerSec = 50;
  public static final double kOuttakePercent = -0.5;

  public static final double kPositionToleranceRotation = 1.0;

  public static final double kPivotP = 20.0;
  public static final double kPivotI = 0.0;
  public static final double kPivotD = 0.0;
  public static final double kPivotV = 0.12;
  public static final double kPivotS = 0.2;
  public static final double kPivotCruiseVelocityRotPerSec = 180;
  public static final double kPivotMaxAccelRotPerSec2 = 230;
  public static final double kPivotStatorCurrentLimit = 60.0;

  public static final double kFeederP = 0.33;
  public static final double kFeederV = 0.12;
  public static final double kFeederStatorCurrentLimit = 150.0;
}
