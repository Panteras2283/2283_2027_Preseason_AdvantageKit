// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.LinearSystemId;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(2), 0.0, SingleJointedArmSim.estimateMOI(0, 0), 0, Math.toRadians(0), Math.toRadians(0), true, Math.toRadians(0));

    private final FlywheelSim feederSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(2), 0, 0), 
        DCMotor.getFalcon500(2), 
        0
    );

    private final ProfiledPIDController leftPivotController = new ProfiledPIDController(5.0, 0, 0, new TrapezoidProfile.Constraints(180, 230));

    private final ProfiledPIDController rightPivotController = new ProfiledPIDController(5.0, 0, 0, new TrapezoidProfile.Constraints(180,230));

    private final PIDController feederController = new PIDController(0.1, 0, 0);

    private double leftTargetRots = 0.0;
    private double rightTargetRots = 0.0;
    private double feederTargetVelocity = 0.0;
    private boolean useDutyCycle = false;
    private double dutyCyclePercent = 0.0;

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        double leftPivotVolts = leftPivotController.calculate(pivotSim.getAngleRads(), leftTargetRots);
        double rightPivotVolts = rightPivotController.calculate(pivotSim.getAngleRads(), rightTargetRots);

        double feederVolts = useDutyCycle ? (dutyCyclePercent * 12.0):
        feederController.calculate(feederSim.getAngularVelocityRPM(), feederTargetVelocity);

        pivotSim.setInputVoltage((leftPivotVolts + rightPivotVolts) / 2.0);
        feederSim.setInputVoltage(feederVolts);

        pivotSim.update(0.02);
        feederSim.update(0.02);

        inputs.leftPivotPositionRots = pivotSim.getAngleRads(); 
        inputs.rightPivotPositionRots = pivotSim.getAngleRads();
        inputs.feederRightVelocityRps = feederSim.getAngularVelocityRPM() / 60.0;
        inputs.feederLeftVelocityRps = feederSim.getAngularVelocityRPM() / 60.0;

        inputs.appliedVolts = new double[]{leftPivotVolts, rightPivotVolts, feederVolts, feederVolts};
        inputs.statorCurrentsAmps = new double[]{
            pivotSim.getCurrentDrawAmps() / 2, pivotSim.getCurrentDrawAmps() / 2, 
            feederSim.getCurrentDrawAmps() / 2, feederSim.getCurrentDrawAmps() / 2
        };
    }

    @Override
    public void setPivotPositions(double leftPos, double rightPos){
        leftTargetRots = leftPos;
        rightTargetRots = rightPos;
    }

    @Override
    public void setFeederVelocity(double velocity){
        useDutyCycle = false;
        feederTargetVelocity = velocity;
    }

    @Override
    public void setFeederDutyCycle(double percent){
        useDutyCycle = true;
        dutyCyclePercent = percent;
    }
}
