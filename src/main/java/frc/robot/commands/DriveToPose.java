// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.tuning.LoggedTunableMeasure;
import frc.robot.util.tuning.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class DriveToPose extends Command {
    private static final ChassisSpeeds zeroSpeeds = new ChassisSpeeds();

    private static final LoggedTunableNumber translationP = new LoggedTunableNumber("DriveToPose/TranslationKP", 5.0);
    private static final LoggedTunableNumber rotationP = new LoggedTunableNumber("DriveToPose/RotationKP", 4.0);
    private static final LoggedTunableMeasure<LinearVelocityUnit, LinearVelocity> maxLinearVelocity =
            new LoggedTunableMeasure<>("DriveToPose/MaxLinearVelocity", MetersPerSecond.of(3.5));
    private static final LoggedTunableMeasure<LinearAccelerationUnit, LinearAcceleration> maxLinearAcceleration =
            new LoggedTunableMeasure<>("DriveToPose/MaxLinearAcceleration", MetersPerSecondPerSecond.of(4.0));
    private static final LoggedTunableMeasure<LinearVelocityUnit, LinearVelocity> slowModeMaxLinearVelocity =
            new LoggedTunableMeasure<>("DriveToPose/SlowModeMaxLinearVelocity", MetersPerSecond.of(2.0));
    private static final LoggedTunableMeasure<LinearAccelerationUnit, LinearAcceleration>
            slowModeMaxLinearAcceleration = new LoggedTunableMeasure<>(
                    "DriveToPose/SlowModeMaxLinearAcceleration", MetersPerSecondPerSecond.of(2.5));
    private static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> maxAngularRate =
            new LoggedTunableMeasure<>("DriveToPose/MaxAngularRate", RadiansPerSecond.of(2.0));
    private static final LoggedTunableMeasure<AngularAccelerationUnit, AngularAcceleration> maxAngularAcceleration =
            new LoggedTunableMeasure<>("DriveToPose/MaxAngularAcceleration", RadiansPerSecondPerSecond.of(4.0));
    private static final LoggedTunableMeasure<DistanceUnit, Distance> translationTolerance =
            new LoggedTunableMeasure<>("DriveToPose/TranslationTolerance", Inches.of(1.0));
    private static final LoggedTunableMeasure<AngleUnit, Angle> rotationTolerance =
            new LoggedTunableMeasure<>("DriveToPose/RotationTolerance", Degrees.of(1.0));
    private static final LoggedTunableMeasure<DistanceUnit, Distance> ffMinRadius =
            new LoggedTunableMeasure<>("DriveToPose/FFMinRadius", Meters.of(0.2));
    private static final LoggedTunableMeasure<DistanceUnit, Distance> ffMaxRadius =
            new LoggedTunableMeasure<>("DriveToPose/FFMaxRadius", Meters.of(0.8));
    private static final LoggedTunableNumber ffMinMultiplier =
            new LoggedTunableNumber("DriveToPose/FFMinMultiplier", 0.5);

    private final Drive drive;
    private final RobotState robotState = RobotState.getInstance();
    private final Supplier<Pose2d> targetPose;
    private final boolean finishAtGoal;
    private final ProfiledPIDController translationController = new ProfiledPIDController(
            translationP.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                    maxLinearVelocity.get().in(MetersPerSecond),
                    maxLinearAcceleration.get().in(MetersPerSecondPerSecond)),
            Constants.mainLoopPeriod.in(Seconds));
    private final ProfiledPIDController rotationController = new ProfiledPIDController(
            rotationP.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                    maxAngularRate.get().in(RadiansPerSecond),
                    maxAngularAcceleration.get().in(RadiansPerSecondPerSecond)),
            Constants.mainLoopPeriod.in(Seconds));
    private final boolean slowMode;

    private Translation2d translationSetpoint = Translation2d.kZero;

    public DriveToPose(Drive drive, Supplier<Pose2d> targetPose, boolean finishAtGoal, boolean slowMode) {
        this.drive = drive;
        this.targetPose = targetPose;
        this.finishAtGoal = finishAtGoal;
        this.slowMode = slowMode;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        translationController.setP(translationP.get());
        rotationController.setP(rotationP.get());
        translationController.setConstraints(new TrapezoidProfile.Constraints(
                (slowMode ? slowModeMaxLinearVelocity : maxLinearVelocity).get().in(MetersPerSecond),
                (slowMode ? slowModeMaxLinearAcceleration : maxLinearAcceleration)
                        .get()
                        .in(MetersPerSecondPerSecond)));
        rotationController.setConstraints(new TrapezoidProfile.Constraints(
                maxAngularRate.get().in(RadiansPerSecond),
                maxAngularAcceleration.get().in(RadiansPerSecondPerSecond)));
        translationController.setTolerance(translationTolerance.get().in(Meters));
        rotationController.setTolerance(rotationTolerance.get().in(Radians));

        var currentPose = robotState.getPose();
        var targetPose = this.targetPose.get();
        var rotationToTarget =
                targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
        var fieldVelocity = robotState.getFieldVelocity();
        var velocityToTarget = ChassisSpeeds.fromFieldRelativeSpeeds(fieldVelocity, rotationToTarget).vxMetersPerSecond;
        var currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        translationController.reset(currentDistance, -velocityToTarget);
        translationController.setTolerance(translationTolerance.get().in(Meters));
        rotationController.reset(currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        rotationController.setTolerance(rotationTolerance.get().in(Radians));
        translationSetpoint = currentPose.getTranslation();
    }

    @Override
    public void execute() {
        var currentPose = robotState.getPose();
        var targetPose = this.targetPose.get();

        var currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        var previousSetpointDistance = translationSetpoint.getDistance(targetPose.getTranslation());
        var ffScalar = MathUtil.clamp(
                (currentDistance - ffMinRadius.get().in(Meters))
                        / (ffMaxRadius.get().in(Meters) - ffMinRadius.get().in(Meters)),
                ffMinMultiplier.get(),
                1.0); // Fudge factor to prevent over-driving
        translationController.reset(
                Math.min(currentDistance, previousSetpointDistance), translationController.getSetpoint().velocity);
        var targetSpeed = translationController.calculate(currentDistance)
                + translationController.getSetpoint().velocity * ffScalar;

        var maxSpeed = Math.min(
                robotState.getMaxSpeed().in(MetersPerSecond),
                maxLinearVelocity.get().in(MetersPerSecond));
        targetSpeed = MathUtil.clamp(targetSpeed, -maxSpeed, maxSpeed);

        var rotationTargetToRobot =
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
        var translationalVelocity = new Translation2d(targetSpeed, 0).rotateBy(rotationTargetToRobot);
        translationSetpoint = new Pose2d(targetPose.getTranslation(), rotationTargetToRobot)
                .transformBy(new Transform2d(translationController.getSetpoint().position, 0.0, Rotation2d.kZero))
                .getTranslation();

        var rotationalVelocity = rotationController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians())
                + rotationController.getSetpoint().velocity;
        if (rotationController.atGoal() && translationController.atGoal()) {
            drive.setRobotSpeeds(zeroSpeeds);
        } else {
            drive.setFieldSpeeds(
                    new ChassisSpeeds(translationalVelocity.getX(), translationalVelocity.getY(), rotationalVelocity));
        }

        Logger.recordOutput(
                "Commands/DriveToPose/SetpointPose",
                new Pose2d(translationSetpoint, Rotation2d.fromRadians(rotationController.getSetpoint().position)));
        Logger.recordOutput("Commands/DriveToPose/TargetPose", targetPose);
        Logger.recordOutput(
                "Commands/DriveToPose/AtGoal", translationController.atGoal() && rotationController.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        drive.setRobotSpeeds(zeroSpeeds);

        Logger.recordOutput("Commands/DriveToPose/SetpointPose", Pose2d.kZero);
        Logger.recordOutput("Commands/DriveToPose/TargetPose", Pose2d.kZero);
    }

    @Override
    public boolean isFinished() {
        return finishAtGoal && atGoal();
    }

    public boolean atGoal() {
        return translationController.atGoal() && rotationController.atGoal();
    }
}
