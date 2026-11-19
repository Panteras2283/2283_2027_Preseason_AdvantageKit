// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO {
  private static final double MOMENT_OF_INERTIA_KG_M2 = 0.004;

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), MOMENT_OF_INERTIA_KG_M2, 1.0),
          DCMotor.getKrakenX60(1));

  private final PIDController controller =
      new PIDController(ShooterConstants.kP, ShooterConstants.kI, 0);

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, ShooterConstants.kV);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private double targetRPS = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              ff.calculate(targetRPS)
                  + controller.calculate(sim.getAngularVelocityRPM() / 60.0, targetRPS),
              -12,
              12);
    }
    sim.setInputVoltage(appliedVolts);
    sim.update(0.02);

    inputs.shooterConnected = true;
    inputs.FlywheelRightRPM = sim.getAngularVelocityRPM();
    inputs.FlywheelLeftRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVelocityRPM(double rpm) {
    closedLoop = true;
    targetRPS = rpm / 60.0;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
