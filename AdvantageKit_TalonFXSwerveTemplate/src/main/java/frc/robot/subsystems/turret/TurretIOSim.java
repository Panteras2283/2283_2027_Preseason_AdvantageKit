// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class TurretIOSim implements TurretIO {
  private static final double MOMENT_OF_INERTIA_KG_M2 = 0.08;

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), MOMENT_OF_INERTIA_KG_M2, TurretConstants.kGearRatio),
          DCMotor.getKrakenX60(1));

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          TurretConstants.kP / TurretConstants.kGearRatio,
          0,
          TurretConstants.kD,
          new TrapezoidProfile.Constraints(
              TurretConstants.kCruiseVelocityRotPerSec, TurretConstants.kMaxAccelRotPerSec2));
  private final SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV);

  private double appliedVolts = 0.0;
  private boolean closedLoop = false;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (closedLoop) {
      var setpoint = controller.getSetpoint();
      appliedVolts =
          ff.calculate(setpoint.velocity) + controller.calculate(sim.getAngularPositionRotations());
    }
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.turretConnected = true;
    inputs.positionRotations = sim.getAngularPositionRotations();
    inputs.velocityRotPerSec = sim.getAngularVelocityRPM() / 60.0;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setPosition(double positionRotations) {
    closedLoop = true;
    controller.setGoal(
        MathUtil.clamp(
            positionRotations,
            TurretConstants.kSoftLimitBackwardsRot,
            TurretConstants.kSoftLimitForwardRot));
  }

  @Override
  public void setOpenLoop(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void resetPosition() {
    sim.setState(0, 0);
  }
}
