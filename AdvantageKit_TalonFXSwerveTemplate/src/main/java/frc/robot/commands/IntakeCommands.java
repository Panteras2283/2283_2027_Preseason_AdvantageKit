// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommands {
  /** Creates a new IntakeCommands. */
  private IntakeCommands() {}

  public static Command defaultCommand(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setPosition(IntakeConstants.LeftFeedPos, IntakeConstants.RightFeedPos);
              intake.stop();
            },
            intake)
        .andThen(Commands.idle(intake));
  }

  public static Command shakeFeeder(Intake intake) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                intake.setPosition(IntakeConstants.LeftShakeUpPos, IntakeConstants.RightShakeUpPos),
            intake),
        Commands.waitUntil(
            () ->
                intake.isAtPosition(
                    IntakeConstants.LeftShakeUpPos, IntakeConstants.RightShakeUpPos)),
        Commands.runOnce(
            () ->
                intake.setPosition(
                    IntakeConstants.LeftShakeDownPos, IntakeConstants.RightShakeDownPos),
            intake),
        Commands.waitUntil(
            () ->
                intake.isAtPosition(
                    IntakeConstants.LeftShakeDownPos, IntakeConstants.RightShakeDownPos)));
  }
}
