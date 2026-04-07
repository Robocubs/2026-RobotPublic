// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.function.Supplier;
import java.util.stream.Stream;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState;
import frc.robot.RobotState.DriveMeasurement;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.drive.DriveIO.DriveIOInputs;
import frc.robot.subsystems.drive.controllers.PathController;
import frc.robot.util.CompositePowerLogger;
import frc.robot.util.PowerLogger;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.subsystems.drive.DriveConstants.kinematics;

public class Drive extends SubsystemBase {
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest =
            new SwerveRequest.FieldCentricFacingAngle();
    private final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();

    private final DriveIO io;
    private final DriveIOInputs driveInputs = new DriveIOInputs();
    private final ModuleIOInputsAutoLogged frontLeftModuleInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged frontRightModuleInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged backLeftModuleInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged backRightModuleInputs = new ModuleIOInputsAutoLogged();
    private final PowerLogger frontLeftDrivePowerLogger = new PowerLogger("FLDrive");
    private final PowerLogger frontRightDrivePowerLogger = new PowerLogger("FRDrive");
    private final PowerLogger backLeftDrivePowerLogger = new PowerLogger("BLDrive");
    private final PowerLogger backRightDrivePowerLogger = new PowerLogger("BRDrive");
    private final PowerLogger frontLeftSteerPowerLogger = new PowerLogger("FLSteer");
    private final PowerLogger frontRightSteerPowerLogger = new PowerLogger("FRSteer");
    private final PowerLogger backLeftSteerPowerLogger = new PowerLogger("BLSteer");
    private final PowerLogger backRightSteerPowerLogger = new PowerLogger("BRSteer");
    private final CompositePowerLogger drivePowerLogger = new CompositePowerLogger(
            "Drive",
            frontLeftDrivePowerLogger,
            frontRightDrivePowerLogger,
            backLeftDrivePowerLogger,
            backRightDrivePowerLogger);
    private final CompositePowerLogger steerPowerLogger = new CompositePowerLogger(
            "Steer",
            frontLeftSteerPowerLogger,
            frontRightSteerPowerLogger,
            backLeftSteerPowerLogger,
            backRightSteerPowerLogger);
    private final RobotState robotState = RobotState.getInstance();
    private final PathController choreoTrajectoryController = new PathController();
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    private final Alert gyroDisconnectedAlert = new Alert("Gyro Disconnected", AlertType.kWarning);

    private @Getter Rotation2d rawHeading = Rotation2d.kZero;
    private @Getter SwerveModuleState[] moduleStates =
            Stream.generate(() -> new SwerveModuleState()).limit(4).toArray(SwerveModuleState[]::new);
    private @Getter SwerveModulePosition[] modulePositions =
            Stream.generate(() -> new SwerveModulePosition()).limit(4).toArray(SwerveModulePosition[]::new);

    public Drive(DriveIO io) {
        this.io = io;

        robotState.setVisionMeasurementConsumer(io::addVisionMeasurements);
        robotState.setPoseResetConsumer(io::resetPose);
        robotState.setHeadingResetConsumer(io::resetRotation);

        pdh.setSwitchableChannel(true);

        // Mitigation for pigeon going dead and needing power cycle to fix
        var gyroConnectedDebouncer = new Debouncer(1.0);
        var initTime = Timer.getTimestamp();
        new Trigger(() -> gyroConnectedDebouncer.calculate(
                        !driveInputs.gyroConnected && Timer.getTimestamp() > initTime + 30.0))
                .onTrue(Commands.runOnce(() -> pdh.setSwitchableChannel(false))
                        .andThen(waitSeconds(2.0))
                        .andThen(() -> pdh.setSwitchableChannel(true))
                        .andThen(waitSeconds(10.0))
                        .ignoringDisable(true)
                        .withName("Drive/RestartGyro"));
    }

    @Override
    public void periodic() {
        io.updateInputs(
                driveInputs,
                frontLeftModuleInputs,
                frontRightModuleInputs,
                backLeftModuleInputs,
                backRightModuleInputs);
        Logger.processInputs("Drive", driveInputs);
        Logger.processInputs("Module/FL", frontLeftModuleInputs);
        Logger.processInputs("Module/FR", frontRightModuleInputs);
        Logger.processInputs("Module/BL", backLeftModuleInputs);
        Logger.processInputs("Module/BR", backRightModuleInputs);

        gyroDisconnectedAlert.set(!driveInputs.gyroConnected);

        frontLeftDrivePowerLogger.log(frontLeftModuleInputs.driveStatorCurrent, frontLeftModuleInputs.driveVoltage);
        frontRightDrivePowerLogger.log(frontRightModuleInputs.driveStatorCurrent, frontRightModuleInputs.driveVoltage);
        backLeftDrivePowerLogger.log(backLeftModuleInputs.driveStatorCurrent, backLeftModuleInputs.driveVoltage);
        backRightDrivePowerLogger.log(backRightModuleInputs.driveStatorCurrent, backRightModuleInputs.driveVoltage);
        frontLeftSteerPowerLogger.log(frontLeftModuleInputs.steerStatorCurrent, frontLeftModuleInputs.steerVoltage);
        frontRightSteerPowerLogger.log(frontRightModuleInputs.steerStatorCurrent, frontRightModuleInputs.steerVoltage);
        backLeftSteerPowerLogger.log(backLeftModuleInputs.steerStatorCurrent, backLeftModuleInputs.steerVoltage);
        backRightSteerPowerLogger.log(backRightModuleInputs.steerStatorCurrent, backRightModuleInputs.steerVoltage);
        drivePowerLogger.log();
        steerPowerLogger.log();

        var measurements = Stream.of(driveInputs.swerve)
                .map(input -> new RobotState.DriveMeasurement(
                        input.timestamp,
                        input.pose,
                        kinematics.toChassisSpeeds(input.moduleStates),
                        kinematics.toChassisSpeeds(input.moduleTargets)))
                .toArray(DriveMeasurement[]::new);
        robotState.addDriveMeasurements(measurements);

        if (driveInputs.swerve.length > 0) {
            var lastInput = driveInputs.swerve[driveInputs.swerve.length - 1];
            rawHeading = lastInput.rawHeading;
            moduleStates = lastInput.moduleStates;
            modulePositions = lastInput.modulePositions;
        }
    }

    public Rotation2d rawHeadingToFieldHeading(Rotation2d heading) {
        return heading.minus(rawHeading).plus(robotState.getHeading());
    }

    public void setRequest(SwerveRequest request) {
        io.setControl(request);
    }

    public void setRobotSpeeds(ChassisSpeeds speeds) {
        io.setControl(applyRobotSpeedsRequest.withSpeeds(speeds));
    }

    public void setFieldSpeeds(ChassisSpeeds speeds) {
        io.setControl(applyFieldSpeedsRequest.withSpeeds(speeds));
    }

    public void setFieldSpeedsFacingHeading(LinearVelocity velocityX, LinearVelocity velocityY, Rotation2d heading) {
        io.setControl(fieldCentricFacingAngleRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withHeadingPID(DriveConstants.rotationP.get(), 0.0, DriveConstants.rotationD.get())
                .withTargetDirection(heading));
    }

    public void followPath(SwerveSample sample) {
        var speeds = choreoTrajectoryController.update(sample, robotState.getPose(), robotState.getMaxSpeed());
        io.setControl(applyRobotSpeedsRequest.withSpeeds(speeds));
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        io.setNeutralMode(neutralMode);
    }

    public DriveToPose toPose(Supplier<Pose2d> poseSupplier, boolean finishAtGoal, boolean slowMode) {
        return new DriveToPose(this, poseSupplier, finishAtGoal, slowMode);
    }

    public Command pointModulesAt(Supplier<Translation2d> target) {
        return run(() -> {
                    var pose = robotState.getPose();
                    var rotationToTarget =
                            target.get().minus(pose.getTranslation()).getAngle();
                    io.setControl(pointRequest.withModuleDirection(rotationToTarget.minus(pose.getRotation())));
                })
                .withName("DrivePointModules");
    }

    public Command pointModules(Supplier<Rotation2d> rotation) {
        return run(() -> io.setControl(pointRequest.withModuleDirection(rotation.get())))
                .withName("DrivePointModules");
    }

    public Command withRobotSpeeds(Supplier<ChassisSpeeds> speeds) {
        return run(() -> setRobotSpeeds(speeds.get())).withName("DriveWithRobotSpeeds");
    }

    public Command withFieldSpeeds(Supplier<ChassisSpeeds> speeds) {
        return run(() -> setFieldSpeeds(speeds.get())).withName("DriveWithFieldSpeeds");
    }

    public Command brake() {
        return run(() -> io.setControl(brakeRequest)).withName("DriveBrake");
    }
}
