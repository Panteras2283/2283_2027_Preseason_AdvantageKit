// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.ShootingPhysics;
import frc.robot.util.ShootingPhysics.AimingSolution;
import frc.robot.util.ShootingTables;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class superstructure extends SubsystemBase {
  public enum State {
    OFF,
    IDLE,
    SHOOTING,
    UNJAM
  }

  private static final Translation2d TURRET_OFFSET = new Translation2d(-0.15, 0.185);

  private static final Translation2d BLUE_TARGET = new Translation2d(4.554, 4.068);
  private static final Translation2d RED_TARGET = new Translation2d(11.9, 4);
  private static final Translation2d BLUE_NSR_TARGET = new Translation2d(2.78, 2.07);
  private static final Translation2d BLUE_NSL_TARGET = new Translation2d(2.78, 6.25);
  private static final Translation2d RED_NSR_TARGET = new Translation2d(13.84, 6.31);
  private static final Translation2d RED_NSL_TARGET = new Translation2d(13.84, 2.07);
  private static final double MIDDLE_Y = 4.00;

  private static final double kTurretLockToleranceDeg = 2.0;
  private static final double kAutoUnjamSeconds = 0.3;
  private static final double kShootPower = 0.85;

  private final Turret turret;
  private final Shooter shooter;
  private final Kicker kicker;
  private final Spindexer spindexer;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedSupplier;
  private final CommandXboxController operator;

  private final Timer unjamTimer = new Timer();

  private Translation2d currentTarget = BLUE_TARGET;
  private AimingSolution solution =
      new AimingSolution(Rotation2d.kZero, 0, new Pose2d(BLUE_TARGET, Rotation2d.kZero));

  private boolean wantsIdle = false;
  private boolean wantsShoot = false;
  private boolean wantsUnjam = false;
  private boolean autoUnjamming = false;
  private boolean shooting = false;
  private boolean hasSpunUp = false;
  private boolean turretLockedOn = false;
  private State state = State.OFF;
  /** Creates a new superstructure. */
  public superstructure(
      Turret turret,
      Shooter shooter,
      Kicker kicker,
      Spindexer spindexer,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedSupplier,
      CommandXboxController operator) {
    this.turret = turret;
    this.shooter = shooter;
    this.kicker = kicker;
    this.spindexer = spindexer;
    this.poseSupplier = poseSupplier;
    this.speedSupplier = speedSupplier;
    this.operator = operator;
  }

  public void toggleIdle() {
    wantsIdle = !wantsIdle;
  }

  public void toggleShooting() {
    wantsShoot = !wantsShoot;
  }

  public boolean isShooting() {
    return shooting;
  }

  public boolean wantsShoot() {
    return wantsShoot;
  }

  public boolean canSHoot() {
    return turretLockedOn;
  }

  public AimingSolution getSolution() {
    return solution;
  }

  @AutoLogOutput(key = "Superstructure/ErrorDegrees")
  public double getErrorDegrees() {
    return turret.getAngle().minus(solution.turretAngle()).getDegrees();
  }

  @Override
  public void periodic() {
    updateTarget();
    solution = calculateAiming();
    turretLockedOn = Math.abs(getErrorDegrees()) < kTurretLockToleranceDeg;

    Logger.recordOutput("Superstructure/VirtualTarget", solution.virtualTarget());
    Logger.recordOutput("Superstructure/State", state);
    Logger.recordOutput("Superstructure/TurretLockedOn", turretLockedOn);
  }

  public Command stateMachineCommand() {
    return Commands.run(
        () -> runStateMachine(-operator.getLeftX() * 10.0),
        this,
        turret,
        shooter,
        kicker,
        spindexer);
  }

  private void updateTarget() {
    Pose2d robotPose = poseSupplier.get();
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      if (robotPose.getX() >= BLUE_TARGET.getX() && robotPose.getY() >= MIDDLE_Y) {
        currentTarget = BLUE_NSL_TARGET;
      } else if (robotPose.getX() >= BLUE_TARGET.getX()) {
        currentTarget = BLUE_NSR_TARGET;
      } else {
        currentTarget = BLUE_TARGET;
      }
    } else {
      if (robotPose.getX() <= RED_TARGET.getX() && robotPose.getY() >= MIDDLE_Y) {
        currentTarget = RED_NSR_TARGET;
      } else if (robotPose.getX() <= RED_TARGET.getX()) {
        currentTarget = RED_NSL_TARGET;
      } else {
        currentTarget = RED_TARGET;
      }
    }
  }

  private AimingSolution calculateAiming() {
    Pose2d robotPose = poseSupplier.get();
    ChassisSpeeds robotSpeeds = speedSupplier.get();
    double rawDistance = robotPose.getTranslation().getDistance(currentTarget);
    double estimatedExitVel = ShootingTables.ExitVelocityMap.get(rawDistance);
    return ShootingPhysics.calculateAimingSolution(
        robotPose, robotSpeeds, TURRET_OFFSET, currentTarget, estimatedExitVel);
  }

  private void runStateMachine(double operatorOffsetDeg) {
    if (wantsShoot && spindexer.isJammed() && !autoUnjamming) {
      autoUnjamming = true;
      unjamTimer.restart();
    }
    if (autoUnjamming && unjamTimer.hasElapsed(kAutoUnjamSeconds)) {
      autoUnjamming = false;
      unjamTimer.stop();
    }
    if (wantsUnjam || autoUnjamming) {
      state = State.UNJAM;
    } else if (wantsShoot) {
      state = State.SHOOTING;
    } else if (wantsIdle) {
      state = State.IDLE;
    } else {
      state = State.OFF;
    }

    switch (state) {
      case OFF -> handleOff();
      case IDLE -> handleIdle();
      case SHOOTING -> handleShooting(operatorOffsetDeg);
      case UNJAM -> handleUnjam();
    }
  }

  private void handleOff() {
    shooting = false;
    hasSpunUp = false;
    turret.setGoal(Rotation2d.kZero);
    shooter.stop();
    kicker.stop();
    spindexer.stop();
  }

  private void handleIdle() {
    shooting = false;
    hasSpunUp = false;
    turret.setGoal(solution.turretAngle());
    shooter.stop();
    kicker.stop();
    spindexer.stop();
  }

  private void handleShooting(double operatorOffsetDeg) {
    shooting = true;

    Rotation2d targetAngle = solution.turretAngle();
    if (Math.abs(operatorOffsetDeg) > 0.05) {
      targetAngle = targetAngle.plus(Rotation2d.fromDegrees(operatorOffsetDeg));
    }
    turret.setGoal(targetAngle);
    shooter.setTargetRPM(true, solution.effectiveDistance());
    if (shooter.isReadyToFire()) {
      hasSpunUp = true;
    }
    boolean locked = turretLockedOn && hasSpunUp;

    if (locked) {
      kicker.kick(kShootPower);
      spindexer.spinClockwise();
    } else {
      kicker.stop();
      spindexer.stop();
    }
  }

  private void handleUnjam() {
    shooting = false;
    hasSpunUp = false;
    shooter.stop();
    kicker.stop();
    spindexer.spinCounterClockwise();
  }
}
