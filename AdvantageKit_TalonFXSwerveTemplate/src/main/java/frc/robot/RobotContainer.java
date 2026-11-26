// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoseCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.kicker.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.spindexer.*;
import frc.robot.subsystems.superstructure.superstructure;
import frc.robot.subsystems.turret.*;
import frc.robot.subsystems.vision.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static final double FEEDING_MAX_SPEED_MPS = 3.5;
  private static final double SHOOTING_MAX_SPEED_MPS = 1.0;

  private static final String LIMELIGHT_NAME = "limelight-three";
  private static final String PHOTON_CAMERA_NAME = "arducam_left";
  private static final Transform3d ROBOT_TO_PHOTON_CAMERA =
    new Transform3d(new Translation3d(-0.34, 0.35025, 0.52), new Rotation3d(0, 0, Math.PI));

  // Controller
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Kicker kicker;
  private final Spindexer spindexer;
  private final Vision vision;
  private final superstructure superstructure;
  private final Turret turret;


  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOReal());
        turret = new Turret(new TurretIOReal());
        shooter = new Shooter(new ShooterIOReal());
        kicker = new Kicker(new KickerIOReal());
        spindexer = new Spindexer(new SpindexerIOReal());
        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
      }
      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOReal());
        turret = new Turret(new TurretIOReal());
        shooter = new Shooter(new ShooterIOReal());
        kicker = new Kicker(new KickerIOReal());
        spindexer = new Spindexer(new SpindexerIOReal());
      }
      default -> {
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIOReal());
        turret = new Turret(new TurretIOReal());
        shooter = new Shooter(new ShooterIOReal());
        kicker = new Kicker(new KickerIOReal());
        spindexer = new Spindexer(new SpindexerIOReal());
      }
    }

    if (Constants.currentMode == Constants.Mode.REAL){
      vision = new Vision(
      drive, 
      new VisionIOLimelight(LIMELIGHT_NAME),
      new VisionIOPhotonVision(PHOTON_CAMERA_NAME, ROBOT_TO_PHOTON_CAMERA));
    }else{
      vision = new Vision(drive, new VisionIOSim(), new VisionIOSim());
    }

    superstructure =
      new superstructure(turret, shooter, kicker, spindexer, drive ::getPose, drive::getChassisSpeeds, operator);

      //Named Commands for Autonomous period
    NamedCommands.registerCommand("Feed", Commands.run(intake::down, intake));
    NamedCommands.registerCommand("toggleShoot", Commands.runOnce(superstructure::toggleShooting));
    NamedCommands.registerCommand("shake feeder", IntakeCommands.shakeFeeder(intake));
    NamedCommands.registerCommand("Feedn't", Commands.runOnce(intake::stop, intake));
    NamedCommands.registerCommand("toggleIdle", Commands.runOnce(superstructure::toggleIdle));
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureBindings();

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
  }

  private double getDynamicMaxSpeed(){
    if (superstructure.isShooting()||superstructure.wantsShoot()){
      return SHOOTING_MAX_SPEED_MPS;
    }
    if(intake.isFeeding()){
      return FEEDING_MAX_SPEED_MPS;
    }
    return drive.getMaxLinearSpeedMetersPerSec();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX(),
            this::getDynamicMaxSpeed));

    superstructure.setDefaultCommand(superstructure.stateMachineCommand());

    intake.setDefaultCommand(IntakeCommands.defaultCommand(intake));

    // Lock to 0° when A button is held
    joystick.a().whileTrue(Commands.run(drive::stopWithX, drive));

    // Switch to X pattern when X button is pressed
    

    // Reset gyro to 0° when B button is pressed
    joystick
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)))
                .ignoringDisable(true));

    joystick.rightBumper().whileTrue(DriveToPoseCommands.preciseDriveToPose(drive, 
    Constants.AutopilotConstants.kPathConstraints, ()-> 0, ()->2, joystick, operator));

    joystick.pov(0).whileTrue(DriveToPoseCommands.fastDriveToPose(drive, Constants.AutopilotConstants.kPathConstraints, ()->1, ()->1));

    operator.leftBumper().onTrue(Commands.runOnce(superstructure::toggleIdle));
    operator.rightBumper().onTrue(Commands.runOnce(superstructure::toggleShooting));

    operator.pov(180).onTrue(Commands.runOnce(intake::down, intake));
    operator.pov(180).onFalse(Commands.runOnce(intake::stop, intake));
    operator.pov(0).onTrue(Commands.runOnce(intake::outtake, intake));
    operator.pov(0).onFalse(Commands.runOnce(intake::stop, intake));
    operator.pov(270).onTrue(Commands.runOnce(intake::up, intake));

    operator.a().whileTrue(IntakeCommands.shakeFeeder(intake).repeatedly());
    operator.a().onFalse(IntakeCommands.defaultCommand(intake));

    operator.x().onTrue(Commands.runOnce(()->superstructure.toggleUnjam(true)));
    operator.x().onFalse(Commands.runOnce(()->superstructure.toggleUnjam(false)));

    operator.y().whileTrue(ShooterCommands.shootOverride(shooter, spindexer, superstructure, kicker));
    operator.start().whileTrue(IntakeCommands.robotUnjam(intake, spindexer));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
