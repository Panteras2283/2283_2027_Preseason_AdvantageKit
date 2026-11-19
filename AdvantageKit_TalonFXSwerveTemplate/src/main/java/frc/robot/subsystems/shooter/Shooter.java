// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShootingTables;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Alert disconnectedAlert =
      new Alert("Disconnected shooter motor.", AlertType.kError);

  private double targetRPM = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    disconnectedAlert.set(!inputs.shooterConnected);
  }

  public void setRPM(boolean forcePrecise, double rpm) {
    targetRPM = rpm;

    if (rpm == 0) {
      io.setVoltage(0);
      return;
    }

    if (!forcePrecise && inputs.FlywheelRightRPM > rpm + ShooterConstants.kOvershootCutoffRPM) {
      io.setVoltage(0);
    } else {
      io.setVelocityRPM(rpm);
    }
  }

  public void setTargetRPM(boolean forcePrecise, double distanceMeters) {
    setRPM(forcePrecise, ShootingTables.FlywheelMap.get(distanceMeters));
  }

  public void stop() {
    setRPM(false, 0);
  }

  @AutoLogOutput(key = "Shooter/ReadyToFire")
  public boolean isReadyToFire() {
    double error = Math.abs(targetRPM - inputs.FlywheelRightRPM);
    boolean withinTolerance = error < (targetRPM * ShooterConstants.kReadyToFireTolerancePercent);
    return withinTolerance && targetRPM > ShooterConstants.kMinRPMToBeReady;
  }
}
