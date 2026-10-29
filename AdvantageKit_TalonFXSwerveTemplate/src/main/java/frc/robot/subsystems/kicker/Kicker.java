// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public boolean shooting = false;
  /** Creates a new Kicker. */
  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
    // This method will be called once per scheduler run
  }

  public void kick(double speed) {
    io.setDutyCycle(speed, speed);
    shooting = true;
  }

  public void stop() {
    io.setDutyCycle(0, 0);
    shooting = false;
  }

  public boolean isShooting() {
    return shooting;
  }
}
