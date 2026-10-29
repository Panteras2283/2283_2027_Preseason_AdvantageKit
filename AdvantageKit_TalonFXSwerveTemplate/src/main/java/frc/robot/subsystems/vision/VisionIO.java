// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean hasEstimate = false;
    public Pose2d estimatedPose = Pose2d.kZero;
    public double timestampSeconds = 0.0;
    public int tagCount = 0;
    public double avgTagDistanceMeters = 0.0;

    public double ambiguity = -1.0;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setRobotHeading(double headingDegrees) {}
}
