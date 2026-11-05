// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.Supplier;

/** Add your docs here. */
public class DriveToPoseCommands {
    private static final double MAX_SPEED_MPS = 4.7;
    private static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Units.rotationsToRadians(0.55);
    private static final double POSE_TOLERANCE_METERS = 0.05;
    private static final double ANGLE_TOLERANCE_RAD = Units.degreesToRadians(2);

    private DriveToPoseCommands() {}

    public static Command fastDriveToPose(
        Drive drive, PathConstraints constraints, Supplier<Integer> side, Supplier<Integer> slot) {
    return new DeferredCommand(
                ()-> {
                    Pose2d scoringPose = Constants.AutopilotConstants.getPose(side.get(), slot.get());
                    return AutoBuilder.pathfindToPose(scoringPose, constraints);
        },
        Set.of(drive));
    }

    public static Command preciseDriveToPose(
      Drive drive,
      PathConstraints constraints,
      Supplier<Integer> side,
      Supplier<Integer> slot,
      CommandXboxController driver,
      CommandXboxController operator) {
    return Commands.sequence(
        new DeferredCommand(
            () -> {
              Pose2d scoringPose = Constants.AutopilotConstants.getPose(side.get(), slot.get());
              Pose2d approachPose =
                  scoringPose.transformBy(new Transform2d(-0.25, 0.0, new Rotation2d()));
              return AutoBuilder.pathfindToPose(approachPose, constraints);
            },
            Set.of(drive)),
        new DeferredCommand(
            () ->
                pidToPose(
                    drive,
                    Constants.AutopilotConstants.getPose(side.get(), slot.get()),
                    driver,
                    operator),
            Set.of(drive)));
  }

  public static Command pidToPose(
      Drive drive, Pose2d target, CommandXboxController driver, CommandXboxController operator) {
    PIDController xController = new PIDController(1.5, 0.5, 0.0);
    PIDController yController = new PIDController(1.5, 0.5, 0.0);
    PIDController rotController = new PIDController(1.5, 0.2, 0.0);
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(POSE_TOLERANCE_METERS);
    yController.setTolerance(POSE_TOLERANCE_METERS);
    rotController.setTolerance(ANGLE_TOLERANCE_RAD);

    return Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              double xOut =
                  MathUtil.clamp(
                      xController.calculate(current.getX(), target.getX()),
                      -MAX_SPEED_MPS,
                      MAX_SPEED_MPS);
              double yOut =
                  MathUtil.clamp(
                      yController.calculate(current.getY(), target.getY()),
                      -MAX_SPEED_MPS,
                      MAX_SPEED_MPS);
              double rotOut =
                  MathUtil.clamp(
                      rotController.calculate(
                          current.getRotation().getRadians(), target.getRotation().getRadians()),
                      -MAX_ANGULAR_SPEED_RAD_PER_SEC,
                      MAX_ANGULAR_SPEED_RAD_PER_SEC);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(xOut, yOut, rotOut, current.getRotation()));

              double rumble =
                  (xController.atSetpoint()
                          && yController.atSetpoint()
                          && rotController.atSetpoint())
                      ? 0.8
                      : 0.0;
              driver.getHID().setRumble(RumbleType.kBothRumble, rumble);
              operator.getHID().setRumble(RumbleType.kBothRumble, rumble);
            },
            drive)
        .beforeStarting(
            () -> {
              xController.reset();
              yController.reset();
              rotController.reset();
            })
        .finallyDo(
            () -> {
              drive.stop();
              driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
              operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            });
        }
}
