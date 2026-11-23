// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Disconnected turret", AlertType.kError);

  private Rotation2d goal = Rotation2d.kZero;

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    disconnectedAlert.set(!inputs.turretConnected);
  }

  public void setGoal(Rotation2d angle) {
    double rotations = MathUtil.inputModulus(angle.getRotations(), -0.5, 0.5);
    rotations =
        MathUtil.clamp(
            rotations,
            TurretConstants.kSoftLimitBackwardsRot,
            TurretConstants.kSoftLimitForwardRot);
    goal = Rotation2d.fromRotations(rotations);
    io.setPosition(rotations);
  }

  public void runCharacterization(double volts) {
    io.setOpenLoop(volts);
  }

  public void resetPosition() {
    io.resetPosition();
  }

  public void stop() {
    io.stop();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(inputs.positionRotations);
  }

  @AutoLogOutput(key = "Turret/TargetDegrees")
  public double getGoalDegrees() {
    return goal.getDegrees();
  }

  @AutoLogOutput(key = "Turret/ErrorDegrees")
  public double getErrorDegrees() {
    return getAngle().minus(goal).getDegrees();
  }

  public boolean atGoal() {
    return Math.abs(getErrorDegrees()) < TurretConstants.kAimToleranceDeg;
  }
}
