// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.characterization;

import java.text.DecimalFormat;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

public class DriveCharacterization {
    public static SysIdCommandSet translationSysId(Drive drive) {
        var request = new SwerveRequest.SysIdSwerveTranslation();
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volts.of(4),
                        null,
                        state -> Logger.recordOutput("SysId/DriveTranslation", state.toString())),
                new SysIdRoutine.Mechanism(output -> drive.setRequest(request.withVolts(output)), null, drive));

        return SysIdCommandSet.builder().sysIdRoutine(routine).build();
    }

    public static SysIdCommandSet steerSysId(Drive drive) {
        var request = new SwerveRequest.SysIdSwerveSteerGains();
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, state -> Logger.recordOutput("SysId/DriveSteer", state.toString())),
                new SysIdRoutine.Mechanism(volts -> drive.setRequest(request.withVolts(volts)), null, drive));

        return SysIdCommandSet.builder().sysIdRoutine(routine).build();
    }

    public static SysIdCommandSet rotationSysId(Drive drive) {
        var request = new SwerveRequest.SysIdSwerveSteerGains();
        var routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, state -> Logger.recordOutput("SysId/DriveRotation", state.toString())),
                new SysIdRoutine.Mechanism(volts -> drive.setRequest(request.withVolts(volts)), null, drive));

        return SysIdCommandSet.builder().sysIdRoutine(routine).build();
    }

    public static Command wheelRadius(Drive drive) {
        var driveBaseRadius = DriveConstants.driveBaseRadius.in(Meters);
        var configuredWheelRadius = DriveConstants.wheelRadius.in(Meters);
        var maxAngularVelocity = 0.5;
        var rampRate = 0.05;
        var limiter = new SlewRateLimiter(rampRate);
        var state = new WheelRadiusCharacterizationState();

        return parallel(
                // Drive control sequence
                sequence(
                        runOnce(() -> limiter.reset(0.0)),
                        drive.withRobotSpeeds(
                                () -> new ChassisSpeeds(0.0, 0.0, limiter.calculate(maxAngularVelocity)))),

                // Measurement sequence
                sequence(
                        waitSeconds(1.0),
                        runOnce(() -> {
                            state.initialPositions = Stream.of(drive.getModulePositions())
                                    .mapToDouble(modulePosition -> modulePosition.distanceMeters)
                                    .toArray();
                            state.lastRawHeading = drive.getRawHeading();
                            state.gyroDelta = 0.0;
                        }),
                        run(() -> {
                                    var rotation = drive.getRawHeading();
                                    state.gyroDelta += Math.abs(
                                            rotation.minus(state.lastRawHeading).getRadians());
                                    state.lastRawHeading = rotation;

                                    var positions = Stream.of(drive.getModulePositions())
                                            .mapToDouble(modulePosition -> modulePosition.distanceMeters)
                                            .toArray();
                                    var wheelDelta = 0.0;
                                    for (var i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.initialPositions[i])
                                                / configuredWheelRadius
                                                / 4.0;
                                    }
                                    state.wheelDelta = wheelDelta;
                                    state.wheelRadius = (state.gyroDelta * driveBaseRadius) / (wheelDelta);

                                    Logger.recordOutput("Characterization/WheelRadius/GyroDelta", state.gyroDelta);
                                    Logger.recordOutput("Characterization/WheelRadius/WheelDelta", state.wheelDelta);
                                    Logger.recordOutput("Characterization/WheelRadius/WheelRadius", state.wheelRadius);
                                })
                                .finallyDo(() -> {
                                    var formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println(
                                            "\tWheel Delta: " + formatter.format(state.wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(state.wheelRadius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(state.wheelRadius))
                                            + " inches");
                                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] initialPositions = new double[4];
        Rotation2d lastRawHeading = Rotation2d.kZero;
        double gyroDelta = 0.0;
        double wheelDelta = 0.0;
        double wheelRadius = 0.0;
    }
}
