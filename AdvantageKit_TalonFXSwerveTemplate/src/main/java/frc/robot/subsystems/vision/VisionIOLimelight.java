// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class VisionIOLimelight implements VisionIO {
    private final String name;

    public VisionIOLimelight(String name){
        this.name = name;
    }

    @Override
    public void setRobotHeading(double headingDegrees){
        LimelightHelpers.SetRobotOrientation(name, headingDegrees, 0, 0, 0, 0, 0);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs){
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        inputs.connected = estimate != null;
        inputs.hasEstimate = estimate != null && estimate.tagCount > 0;
        inputs.ambiguity = -1.0; // not exposed by MegaTag2

        if (inputs.hasEstimate) {
          inputs.estimatedPose = estimate.pose;
          inputs.timestampSeconds = estimate.timestampSeconds;
          inputs.tagCount = estimate.tagCount;
          inputs.avgTagDistanceMeters = estimate.avgTagDist;
        }
    }
}
