// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.superstructure.superstructure;

/** Add your docs here. */
public class ShooterCommands {
    public ShooterCommands(){}
        public static Command shootOverride(Shooter shooter, Spindexer spindexer, superstructure superstructure, Kicker kicker){
            return Commands.run(
                ()->{
                    shooter.setTargetRPM(true, superstructure.getSolution().effectiveDistance());
                    kicker.kick(0.85);
                    spindexer.spinClockwise();
                }, 
                shooter,
                spindexer,
                kicker);
        }

        public static Command simpleShoot(
            Shooter shooter, Kicker kicker, Spindexer spindexer, double rpm){
        return Commands.run(
            ()-> {
                shooter.setRPM(true, rpm);
                kicker.kick(0.85);
                spindexer.spinClockwise();
            },
            shooter,
            kicker,
            spindexer);
        }
        
        
}
