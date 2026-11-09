// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SpindexerIOSim implements SpindexerIO {
    private static final double MOMENT_OF_INERTIA_KG_M2 = 0.001;

    private final DCMotorSim sim = 
    new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), MOMENT_OF_INERTIA_KG_M2, 1.0),
    DCMotor.getNeoVortex(1));

    private final PIDController controller = new PIDController(SpindexerConstants.kP, SpindexerConstants.kI, SpindexerConstants.kD);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, SpindexerConstants.kFF);

    private double appliedVolts = 0.0;
    private double targetRPM = 0.0;
    private boolean closedLoop = false;

    @Override
    public void updateInputs(SpindexerIOInputs inputs){
        if (closedLoop) {
            appliedVolts = MathUtil.clamp(ff.calculate(targetRPM) + controller.calculate(sim.getAngularVelocityRPM(), targetRPM),
            -12,
            12);
        }
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);

        inputs.connected = true;
        inputs.velocityRPM = sim.getAngularVelocityRPM();
        inputs.appliedOutput = appliedVolts / 12.0;
        inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
    }

    @Override
    public void setVelocityRPM(double rpm) {
        closedLoop = true;
        targetRPM = rpm;
    }

    @Override
    public void stop(){
        closedLoop = false;
        appliedVolts = 0;
    }
}

