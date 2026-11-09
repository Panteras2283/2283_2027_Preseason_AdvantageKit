// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Add your docs here. */
public class SpindexerIOReal implements SpindexerIO{
    private final SparkFlex SpindexerMotor = new SparkFlex(SpindexerConstants.SpindexerMotorID, MotorType.kBrushless);
    private final SparkClosedLoopController closedLoopController = SpindexerMotor.getClosedLoopController();
    private final RelativeEncoder encoder = SpindexerMotor.getEncoder();

    public SpindexerIOReal() {
        var config = new SparkFlexConfig();
        config.closedLoop.pid(SpindexerConstants.kP, SpindexerConstants.kI, SpindexerConstants.kD);
        config.closedLoop.feedForward.kV(SpindexerConstants.kFF);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        SpindexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(SpindexerIOInputs inputs){
        inputs.connected = true;
        inputs.velocityRPM = encoder.getVelocity();
        inputs.appliedOutput = SpindexerMotor.getAppliedOutput();
        inputs.currentAmps = SpindexerMotor.getOutputCurrent();
    }

    @Override
    public void setVelocityRPM(double rpm){
        closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void stop(){
        SpindexerMotor.set(0);
    }
}
