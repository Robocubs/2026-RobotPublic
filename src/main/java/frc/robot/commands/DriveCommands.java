// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.controllers.SmoothDriveController;
import frc.robot.util.tuning.LoggedTunableMeasure;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class DriveCommands {
    private static final LoggedTunableMeasure<TimeUnit, Time> headingHoldTimeout =
            new LoggedTunableMeasure<>("JoystickDrive/HeadingHoldTimeout", Seconds.of(0.25));

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        x *= RobotState.isBlue() ? 1.0 : -1.0;
        y *= RobotState.isBlue() ? 1.0 : -1.0;

        var linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.translationDeadband);
        var linearDirection = new Rotation2d(Math.atan2(y, x));

        linearMagnitude = linearMagnitude * linearMagnitude * DriveConstants.maxSpeed.in(MetersPerSecond);

        return new Translation2d(linearMagnitude, linearDirection);
    }

    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier smoothShooting,
            BooleanSupplier smoothPassing) {
        var state = new JoystickDriveState();
        var smoothController = new SmoothDriveController();

        return drive.runOnce(() -> {
                    state.joystickLastTouched = Timer.getTimestamp();
                    state.headingSetpoint = Optional.empty();
                    smoothController.resetFromMeasuredValues();
                })
                .andThen(Commands.run(() -> {
                    smoothController.updateLoopTime();
                    var useSmoothShooting = smoothShooting.getAsBoolean();
                    var useSmoothPassing = smoothPassing.getAsBoolean();
                    var useSmooth = useSmoothShooting || useSmoothPassing;
                    if (!useSmooth) {
                        smoothController.updateFromMeasuredValues();
                    }

                    var linearVelocity =
                            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    var omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveConstants.rotationDeadband);

                    if (useSmoothShooting) {
                        linearVelocity = smoothController.smoothShootingLinearVelocity(linearVelocity);
                    } else if (useSmoothPassing) {
                        linearVelocity = smoothController.smoothPassingLinearVelocity(linearVelocity);
                    }

                    var rotationDemandIsZero = MathUtil.isNear(0.0, omega, Constants.epsilon);
                    if (!rotationDemandIsZero) {
                        state.joystickLastTouched = Timer.getTimestamp();
                    }

                    var headingLocked = rotationDemandIsZero
                            && Timer.getTimestamp()
                                    > state.joystickLastTouched
                                            + headingHoldTimeout.get().in(Seconds);
                    Logger.recordOutput("Commands/JoystickDrive/HeadingLocked", headingLocked);

                    if (headingLocked) {
                        // Capture the heading once when heading-hold first engages
                        if (state.headingSetpoint.isEmpty()) {
                            state.headingSetpoint = Optional.of(drive.getRawHeading());
                        }

                        drive.setFieldSpeedsFacingHeading(
                                MetersPerSecond.of(linearVelocity.getX()),
                                MetersPerSecond.of(linearVelocity.getY()),
                                drive.rawHeadingToFieldHeading(state.headingSetpoint.get()));

                        Logger.recordOutput("Commands/JoystickDrive/HeadingSetpoint", state.headingSetpoint.get());

                        return;
                    }

                    state.headingSetpoint = Optional.empty();
                    omega = Math.copySign(omega * omega, omega)
                            * DriveConstants.maxAngularVelocity.in(RadiansPerSecond);

                    if (useSmoothShooting) {
                        omega = smoothController.smoothShootingAngularVelocity(omega);
                    } else if (useSmoothPassing) {
                        omega = smoothController.smoothPassingAngularVelocity(omega);
                    }

                    drive.setFieldSpeeds(new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega));
                }))
                .withName("JoystickDrive");
    }

    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier,
            BooleanSupplier smoothShooting,
            BooleanSupplier smoothPassing) {
        var smoothController = new SmoothDriveController();

        return drive.runOnce(() -> {
                    smoothController.resetFromMeasuredValues();
                })
                .andThen(drive.run(() -> {
                    smoothController.updateLoopTime();

                    var linearVelocity =
                            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    var headingGoal = rotationSupplier.get();

                    if (smoothShooting.getAsBoolean()) {
                        linearVelocity = smoothController.smoothShootingLinearVelocity(linearVelocity);
                    } else if (smoothPassing.getAsBoolean()) {
                        linearVelocity = smoothController.smoothPassingLinearVelocity(linearVelocity);
                    } else {
                        smoothController.updateFromMeasuredValues();
                    }

                    drive.setFieldSpeedsFacingHeading(
                            MetersPerSecond.of(linearVelocity.getX()),
                            MetersPerSecond.of(linearVelocity.getY()),
                            headingGoal);

                    Logger.recordOutput("Commands/JoystickDrive/HeadingGoal", headingGoal);
                }))
                .withName("JoystickDriveAtAngle");
    }

    public static Command oscillate(Drive drive) {
        var smoothController = new SmoothDriveController();
        var omegaTarget = 1.5;
        Function<Double, Command> turn = (Double omega) -> drive.run(() -> {
                    smoothController.updateLoopTime();
                    var smoothed = smoothController.smoothPassingAngularVelocity(omega);
                    drive.setRobotSpeeds(new ChassisSpeeds(0, 0, smoothed));
                })
                .withTimeout(0.25);
        return drive.runOnce(() -> {
                    smoothController.resetFromMeasuredValues();
                })
                .andThen(Commands.repeatingSequence(turn.apply(omegaTarget), turn.apply(-omegaTarget)))
                .withName("DriveOscillate");
    }

    private static class JoystickDriveState {
        Optional<Rotation2d> headingSetpoint = Optional.empty();
        double joystickLastTouched = 0.0;
    }
}
