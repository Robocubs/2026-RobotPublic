// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autonomous.AutoRoutines;
import frc.robot.autonomous.LoggedAutoChooser;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.characterization.DriveCharacterization;
import frc.robot.simulation.SimRobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOReplay;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.flywheel.FlywheelIO;
import frc.robot.subsystems.superstructure.flywheel.FlywheelIOHardware;
import frc.robot.subsystems.superstructure.flywheel.FlywheelIOSim;
import frc.robot.subsystems.superstructure.hood.HoodIO;
import frc.robot.subsystems.superstructure.hood.HoodIOHardware;
import frc.robot.subsystems.superstructure.hood.HoodIOSim;
import frc.robot.subsystems.superstructure.indexer.IndexerIO;
import frc.robot.subsystems.superstructure.indexer.IndexerIOHardware;
import frc.robot.subsystems.superstructure.indexer.IndexerIOSim;
import frc.robot.subsystems.superstructure.kicker.KickerIO;
import frc.robot.subsystems.superstructure.kicker.KickerIOHardware;
import frc.robot.subsystems.superstructure.kicker.KickerIOSim;
import frc.robot.subsystems.superstructure.turret.TurretIO;
import frc.robot.subsystems.superstructure.turret.TurretIOHardware;
import frc.robot.subsystems.superstructure.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagIO;
import frc.robot.subsystems.vision.apriltag.AprilTagIOHardware;
import frc.robot.subsystems.vision.apriltag.AprilTagIOSim;

import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    protected final Drive drive;
    protected final Vision vision;
    protected final Intake intake;
    protected final Superstructure superstructure;

    // Controller
    private final CommandXboxController driverController = new CommandXboxController(0);

    // Other
    private final RobotState robotState = RobotState.getInstance();
    Optional<SimRobotState> simRobotState = Optional.empty();

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(new DriveIOHardware());
                vision = new Vision(new AprilTagIO[] {new AprilTagIOHardware(VisionConstants.frontAprilTagConfig)});
                intake = new Intake(new IntakeIOHardware());
                superstructure = new Superstructure(
                        new FlywheelIOHardware(),
                        new HoodIOHardware(),
                        new TurretIOHardware(),
                        new IndexerIOHardware(),
                        new KickerIOHardware());
                break;
            case SIM:
                var simRobotState = new SimRobotState();
                this.simRobotState = Optional.of(simRobotState);
                drive = new Drive(new DriveIOSim(simRobotState));
                vision = new Vision(
                        new AprilTagIO[] {new AprilTagIOSim(VisionConstants.frontAprilTagConfig, simRobotState)});
                intake = new Intake(new IntakeIOSim(simRobotState));
                superstructure = new Superstructure(
                        new FlywheelIOSim(simRobotState),
                        new HoodIOSim(simRobotState),
                        new TurretIOSim(simRobotState),
                        new IndexerIOSim(simRobotState),
                        new KickerIOSim(simRobotState) {});
                break;
            default:
                drive = new Drive(new DriveIOReplay());
                vision = new Vision(new AprilTagIO[] {() -> VisionConstants.frontAprilTagConfig});
                intake = new Intake(new IntakeIO() {});
                superstructure = new Superstructure(
                        new FlywheelIO() {}, new HoodIO() {}, new TurretIO() {}, new IndexerIO() {}, new KickerIO() {});
                break;
        }

        SmartDashboard.putData(drive);
        SmartDashboard.putData(intake);
        SmartDashboard.putData(superstructure);

        // Set up auto routines
        configureAutoRoutines();
        configureButtonBindings();
    }

    private void configureAutoRoutines() {
        var autoChooser = new LoggedAutoChooser("Autonomous");
        RobotModeTriggers.autonomous()
                .whileTrue(autoChooser.selectedCommandScheduler().withName("Autonomous"));

        var autoRoutines = new AutoRoutines(drive);

        autoChooser.addRoutine("Demo", autoRoutines::demo);

        if (Constants.characterizationEnabled) {
            var characterizationChooser = new LoggedAutoChooser("Characterization");
            autoChooser.addCmd("Characterization", characterizationChooser::selectedCommandScheduler);

            characterizationChooser.addCmd("Drive Wheel Radius", () -> DriveCharacterization.wheelRadius(drive));
            DriveCharacterization.translationSysId(drive).addAutos(characterizationChooser, "Drive Translation");
            DriveCharacterization.steerSysId(drive).addAutos(characterizationChooser, "Drive Steer");
            DriveCharacterization.rotationSysId(drive).addAutos(characterizationChooser, "Drive Rotation");

            superstructure.flywheelSysId().addAutos(characterizationChooser, "Superstructure Flywheel");
            superstructure.hoodSysId().addAutos(characterizationChooser, "Superstructure Hood");
            superstructure.turretSysId().addAutos(characterizationChooser, "Superstructure Turret");
            superstructure.indexerSysId().addAutos(characterizationChooser, "Superstructure Indexer");
            superstructure.kickerSysId().addAutos(characterizationChooser, "Superstructure Kicker");

            intake.extensionSysId().addAutos(characterizationChooser, "Intake Extension");
        }
    }

    private void configureButtonBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Default commands
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                driverController.rightTrigger().or(driverController.leftTrigger())));

        intake.setDefaultCommand(intake.stop());

        superstructure.setDefaultCommand(superstructure.stop());

        driverController.rightBumper().whileTrue(drive.brake());

        driverController
                .start()
                .onTrue(runOnce(() ->
                                robotState.resetHeading(RobotState.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg))
                        .withName("ResetHeading"));

        driverController.back().onTrue(superstructure.zero());

        driverController.rightTrigger().whileTrue(superstructure.shoot());
        driverController
                .leftTrigger()
                .and(() -> !robotState.inAllianceZone() && !robotState.onAllianceBump())
                .whileTrue(superstructure.feed());

        driverController.y().onTrue(intake.agitate());
        driverController.a().onTrue(intake.stop());
        driverController.x().onTrue(intake.intakeEfficient());
        driverController.b().onTrue(intake.stow());

        // Dashboard bindings
        SmartDashboard.putData("ZeroIntake", intake.zeroRoutine());
    }
}
