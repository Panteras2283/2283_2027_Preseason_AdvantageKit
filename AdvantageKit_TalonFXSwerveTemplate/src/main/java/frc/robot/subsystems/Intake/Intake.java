// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private boolean feeding = false;
  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    // This method will be called once per scheduler run
  }

  public void setPosition(double leftRotations, double rightRotations) {
    io.setPivotPositions(leftRotations, rightRotations);
  }

  public void down() {
    feeding = true;
    io.setPivotPositions(IntakeConstants.LeftFeedPos, IntakeConstants.RightFeedPos);
    io.setFeederVelocity(IntakeConstants.kFeederVelocityRotPerSec);
  }

  public void feed() {
    feeding = true;
    io.setFeederVelocity(IntakeConstants.kFeederVelocityRotPerSec);
  }

  public void up() {
    feeding = false;
    io.setPivotPositions(IntakeConstants.LeftUpPos, IntakeConstants.RightUpPos);
  }

  public void outtake() {
    feeding = false;
    io.setFeederDutyCycle(IntakeConstants.kOuttakePercent);
  }

  public void stop() {
    feeding = false;
    io.setFeederDutyCycle(0);
  }

  public boolean isFeeding() {
    return feeding;
  }

  public double getLeftPosition() {
    return inputs.leftPivotPositionRots;
  }

  public double getRightPosition() {
    return inputs.rightPivotPositionRots;
  }

  public boolean isAtPosition(double leftTarget, double rightTarget) {
    return Math.abs(getLeftPosition() - leftTarget) <= IntakeConstants.kPositionToleranceRotation
        && Math.abs(getRightPosition() - rightTarget) <= IntakeConstants.kPositionToleranceRotation;
  }
}
