// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  /** Creates a new Spindexer. */
  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
  }

  public void spin(double rpm) {
    io.setVelocityRPM(rpm);
  }

  public void spinClockwise() {
    spin(SpindexerConstants.kSpinClockwiseRPM);
  }

  public void spinCounterClockwise() {
    spin(SpindexerConstants.kSpinCounterclockwiseRPM);
  }

  public void unjam() {
    spin(SpindexerConstants.kUnjamRPM);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Spindexer/Jammed")
  public boolean isJammed() {
    return Math.abs(inputs.currentAmps) > SpindexerConstants.kJamCurrentAmps
        && Math.abs(inputs.velocityRPM) < SpindexerConstants.kJamMaxRPM;
  }
}
