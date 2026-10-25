// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureSetpoint;
import frc.robot.subsystems.superstructure.flywheel.FlywheelConstants;
import frc.robot.subsystems.superstructure.turret.Turret.TurretSetpoint;
import frc.robot.subsystems.superstructure.turret.TurretConstants;
import frc.robot.util.PhysicsUtil;
import frc.robot.util.tuning.LoggedTunableBoolean;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SuperstructureSetpointCalculator {
    private static final LoggedTunableBoolean turretRotationalVelocityCompensation =
            new LoggedTunableBoolean("SuperstructureSetpointCalculator/TurretRotationalVelocityCompensation", true);

    private static final InterpolatingDoubleTreeMap flywheelMap = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.17, 245.0),
            Map.entry(1.84, 260.0),
            Map.entry(2.75, 275.0),
            Map.entry(3.66, 285.0),
            Map.entry(4.50, 300.0),
            Map.entry(5.30, 317.5),
            Map.entry(6.20, 335.0));
    private static final InterpolatingDoubleTreeMap hoodMap = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.17, 0.25),
            Map.entry(1.84, 0.35),
            Map.entry(2.75, 0.47),
            Map.entry(3.66, 0.6),
            Map.entry(4.50, 0.7),
            Map.entry(5.30, 0.75),
            Map.entry(6.20, 0.85));
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.17, 1.00),
            Map.entry(1.84, 1.05),
            Map.entry(2.75, 1.08),
            Map.entry(3.66, 1.13),
            Map.entry(4.50, 1.05),
            Map.entry(5.30, 1.05),
            Map.entry(6.20, 1.10));

    private static final InterpolatingDoubleTreeMap passingFlywheelVelocityMap =
            InterpolatingDoubleTreeMap.ofEntries(Map.entry(3.0, 200.0), Map.entry(14.0, 550.0));
    private static final Angle passingHoodAngle = Degrees.of(50);
    private static final Angle neutralTrackingHoodAngle = Degrees.of(30);

    private static final AngularVelocity idleFlywheelVelocity = RadiansPerSecond.of(200.0);

    private static final double bluePassingPointX = FieldConstants.blueAllianceThreshold - 1.5;
    private static final double redPassingPointX = FieldConstants.redAllianceThreshold + 1.5;

    private static final double passingYMinOffset = 1.5;
    private static final double passingYMaxOffset = 1.75;

    private static final double hubOffset = 0.3;
    private static final Translation2d blueHubFL = new Translation2d(
            FieldConstants.blueAllianceThreshold - hubOffset, FieldConstants.blueHubLeftThresholdY + hubOffset);
    private static final Translation2d blueHubFR = new Translation2d(
            FieldConstants.blueAllianceThreshold - hubOffset, FieldConstants.blueHubRightThresholdY - hubOffset);
    private static final Translation2d blueHubBL = new Translation2d(
            FieldConstants.blueNeutralZoneStart + hubOffset, FieldConstants.blueHubLeftThresholdY + hubOffset);
    private static final Translation2d blueHubBR = new Translation2d(
            FieldConstants.blueNeutralZoneStart + hubOffset, FieldConstants.blueHubRightThresholdY - hubOffset);
    private static final Translation2d redHubFL = new Translation2d(
            FieldConstants.redAllianceThreshold + hubOffset, FieldConstants.redHubLeftThresholdY - hubOffset);
    private static final Translation2d redHubFR = new Translation2d(
            FieldConstants.redAllianceThreshold + hubOffset, FieldConstants.redHubRightThresholdY + hubOffset);
    private static final Translation2d redHubBL = new Translation2d(
            FieldConstants.redNeutralZoneStart - hubOffset, FieldConstants.redHubLeftThresholdY - hubOffset);
    private static final Translation2d redHubBR = new Translation2d(
            FieldConstants.redNeutralZoneStart - hubOffset, FieldConstants.redHubRightThresholdY + hubOffset);

    private final RobotState robotState = RobotState.getInstance();
    private final Supplier<ChassisSpeeds> fieldVelocity = robotState::getFieldVelocity;

    private PassingTarget selectedTarget = PassingTarget.NONE;

    public SuperstructureSetpoint getSetpointForTrackingPose() {
        var pose = robotState.mostRecentlyInOpponentZone() ? robotState.getOpposingHubPose() : robotState.getHubPose();
        var turretSetpoint = getTurretSetpointFor(pose.getTranslation());

        if (robotState.mostRecentlyInAllianceZone()) {
            var shootingSetpoint = getSetpointForShooting();
            return new SuperstructureSetpoint(
                    shootingSetpoint.flywheelVelocity(),
                    shootingSetpoint.hoodAngle(),
                    turretSetpoint.position(),
                    turretSetpoint.velocity());
        }

        return new SuperstructureSetpoint(
                idleFlywheelVelocity, neutralTrackingHoodAngle, turretSetpoint.position(), turretSetpoint.velocity());
    }

    public SuperstructureSetpoint getSetpointForShooting() {
        var adjustedHubTranslation = adjustTranslationForRobotMovement(
                robotState.getHubPose().getTranslation(),
                timeOfFlightMap.get(robotState.getDistanceToHub().in(Meters)));

        var currentTurretTranslation = robotState.getTurretPose().toPose2d().getTranslation();
        var turretDistanceFromHub =
                adjustedHubTranslation.minus(currentTurretTranslation).getNorm();

        var turretSetpoint = getTurretSetpointFor(adjustedHubTranslation);
        var flywheelVelocity = flywheelMap.get(turretDistanceFromHub);
        var hoodAngle = hoodMap.get(turretDistanceFromHub);

        Logger.recordOutput(
                "SuperstructureSetpointCalculator/AdjustedHubPose",
                new Pose2d(adjustedHubTranslation, Rotation2d.kZero));

        return new SuperstructureSetpoint(
                RadiansPerSecond.of(flywheelVelocity),
                Radians.of(hoodAngle),
                turretSetpoint.position(),
                turretSetpoint.velocity());
    }

    public SuperstructureSetpoint getSetpointForPassing() {
        var passingPose = getPassingPose();
        var distanceToPassingPose =
                robotState.getTurretDistanceFrom(passingPose).in(Meters);

        var adjustedPassingTranslation = adjustTranslationForRobotMovement(
                passingPose.getTranslation(), calculateTheoreticalPassingTimeOfFlight(distanceToPassingPose));

        var currentTurretTranslation = robotState.getTurretPose().toPose2d().getTranslation();
        var adjustedDistance =
                adjustedPassingTranslation.minus(currentTurretTranslation).getNorm();

        var flywheelVelocity = passingFlywheelVelocityMap.get(adjustedDistance);
        var turretSetpointForPassing = getTurretSetpointFor(adjustedPassingTranslation);

        Logger.recordOutput("SuperstructureSetpointCalculator/DistanceToPassingPose", distanceToPassingPose);
        Logger.recordOutput("SuperstructureSetpointCalculator/PassingPose", passingPose);
        Logger.recordOutput(
                "SuperstructureSetpointCalculator/AdjustedPassingPose",
                new Pose2d(adjustedPassingTranslation, passingPose.getRotation()));
        return new SuperstructureSetpoint(
                RadiansPerSecond.of(flywheelVelocity),
                passingHoodAngle,
                turretSetpointForPassing.position(),
                turretSetpointForPassing.velocity());
    }

    public void resetPassingTarget() {
        selectedTarget = PassingTarget.NONE;
    }

    private PassingTarget getClosestPassingTarget() {
        var turretTranslation = robotState.getTurretPose().toPose2d().getTranslation();
        var x = RobotState.isBlue() ? bluePassingPointX : redPassingPointX;
        var leftTarget = new Translation2d(x, computePassingY(PassingTarget.LEFT));
        var rightTarget = new Translation2d(x, computePassingY(PassingTarget.RIGHT));
        return turretTranslation.getDistance(leftTarget) <= turretTranslation.getDistance(rightTarget)
                ? PassingTarget.LEFT
                : PassingTarget.RIGHT;
    }

    private Pose2d getPassingPose() {
        var turretY = robotState.getTurretPose().getY();

        if (turretY >= FieldConstants.blueHubLeftThresholdY) {
            selectedTarget = PassingTarget.LEFT;
        } else if (turretY <= FieldConstants.blueHubRightThresholdY) {
            selectedTarget = PassingTarget.RIGHT;
        } else if (selectedTarget == PassingTarget.NONE) {
            selectedTarget = getClosestPassingTarget();
        }

        var other = selectedTarget == PassingTarget.LEFT ? PassingTarget.RIGHT : PassingTarget.LEFT;
        if (isPassingBlockedFor(selectedTarget) && !isPassingBlockedFor(other)) {
            selectedTarget = other;
        }

        var x = RobotState.isBlue() ? bluePassingPointX : redPassingPointX;
        var y = computePassingY(selectedTarget);
        var rotation = RobotState.isBlue() ? Rotation2d.kZero : Rotation2d.k180deg;

        Logger.recordOutput("SuperstructureSetpointCalculator/SelectedPassingTarget", selectedTarget.toString());
        return new Pose2d(new Translation2d(x, y), rotation);
    }

    private double computePassingY(PassingTarget target) {
        var turretY = robotState.getTurretPose().getY();
        if (target == PassingTarget.LEFT) {
            return MathUtil.clamp(
                    turretY,
                    FieldConstants.blueHubLeftThresholdY + passingYMinOffset,
                    FieldConstants.blueHubLeftThresholdY + passingYMaxOffset);
        } else {
            return MathUtil.clamp(
                    turretY,
                    FieldConstants.blueHubRightThresholdY - passingYMaxOffset,
                    FieldConstants.blueHubRightThresholdY - passingYMinOffset);
        }
    }

    private Translation2d adjustTranslationForRobotMovement(Translation2d originalTranslation, double timeOfFlight) {
        // Adjust the position by the negative of turret velocity during time of flight.
        // The turret's field-frame velocity includes the robot's translational velocity plus
        // the tangential velocity from rotation: v_turret = v_robot + omega x r_turret_field.
        var velocity = fieldVelocity.get();
        var turretVelocityX = velocity.vxMetersPerSecond;
        var turretVelocityY = velocity.vyMetersPerSecond;
        if (turretRotationalVelocityCompensation.get()) {
            var turretOffsetField =
                    TurretConstants.robotToTurret2d.getTranslation().rotateBy(robotState.getHeading());
            var omega = velocity.omegaRadiansPerSecond;
            turretVelocityX -= omega * turretOffsetField.getY();
            turretVelocityY += omega * turretOffsetField.getX();
        }

        return new Translation2d(
                originalTranslation.getX() - turretVelocityX * timeOfFlight,
                originalTranslation.getY() - turretVelocityY * timeOfFlight);
    }

    private TurretSetpoint getTurretSetpointFor(Translation2d poseTranslation) {
        var robotPoseNextLoop = PhysicsUtil.predictPoseFromFieldSpeeds(
                robotState.getPose(), fieldVelocity.get(), Constants.mainLoopPeriod);
        var predictedTurretTranslation =
                robotPoseNextLoop.transformBy(TurretConstants.robotToTurret2d).getTranslation();
        var turretHeading = poseTranslation.minus(predictedTurretTranslation).getAngle();
        var targetTurretPosition = turretHeading.minus(robotPoseNextLoop.getRotation());

        return new TurretSetpoint(
                Radians.of(targetTurretPosition.getRadians()),
                RadiansPerSecond.of(-fieldVelocity.get().omegaRadiansPerSecond));
    }

    public boolean isPassingBlocked() {
        var target = selectedTarget != PassingTarget.NONE ? selectedTarget : getClosestPassingTarget();
        return isPassingBlockedFor(target);
    }

    private boolean isPassingBlockedFor(PassingTarget target) {
        var turretTranslation = robotState.getTurretPose().toPose2d().getTranslation();
        var x = RobotState.isBlue() ? bluePassingPointX : redPassingPointX;
        var passingTarget = new Translation2d(x, computePassingY(target));

        return isBlockedByHub(turretTranslation, passingTarget, blueHubFL, blueHubFR, blueHubBL, blueHubBR)
                || isBlockedByHub(turretTranslation, passingTarget, redHubFL, redHubFR, redHubBL, redHubBR);
    }

    private static boolean isBlockedByHub(
            Translation2d from,
            Translation2d to,
            Translation2d fl,
            Translation2d fr,
            Translation2d bl,
            Translation2d br) {
        return segmentsIntersect(from, to, bl, br)
                || segmentsIntersect(from, to, fl, bl)
                || segmentsIntersect(from, to, fr, br);
    }

    private static boolean segmentsIntersect(Translation2d p1, Translation2d p2, Translation2d p3, Translation2d p4) {
        var d1x = p2.getX() - p1.getX();
        var d1y = p2.getY() - p1.getY();
        var d2x = p4.getX() - p3.getX();
        var d2y = p4.getY() - p3.getY();

        var cross = d1x * d2y - d1y * d2x;
        if (Math.abs(cross) < 1e-10) {
            return false; // parallel
        }

        var dx = p3.getX() - p1.getX();
        var dy = p3.getY() - p1.getY();
        var t = (dx * d2y - dy * d2x) / cross;
        var u = (dx * d1y - dy * d1x) / cross;

        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }

    private double calculateTheoreticalPassingTimeOfFlight(double x) {
        return calculateTheoreticalTimeOfFlight(
                x,
                passingHoodAngle.in(Radians),
                TurretConstants.robotToTurret.getZ() + FlywheelConstants.turretToFlywheel.getZ(),
                0.0);
    }

    private double calculateTheoreticalTimeOfFlight(double x, double theta, double y0, double y) {
        // t = sqrt(2(y0 + x tan(theta) - y) / g)
        // impossible if discriminant (term under sqrt) is negative
        var discriminant = 2 * (y0 + x * Math.tan(theta) - y) / 9.81;
        if (discriminant < 1.0) {
            return 1.0;
        }

        return Math.sqrt(discriminant);
    }

    private enum PassingTarget {
        NONE,
        LEFT,
        RIGHT
    }
}
