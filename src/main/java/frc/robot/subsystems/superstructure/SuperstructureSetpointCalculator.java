// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureSetpoint;
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
            Map.entry(1.17, 250.0),
            Map.entry(1.84, 265.0),
            Map.entry(2.75, 280.0),
            Map.entry(3.66, 290.0),
            Map.entry(4.50, 305.0),
            Map.entry(5.30, 322.5),
            Map.entry(6.20, 340.0));
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

    private final RobotState robotState = RobotState.getInstance();
    private final Supplier<ChassisSpeeds> fieldVelocity = robotState::getTargetFieldVelocity;

    public SuperstructureSetpoint getSetpointForHub() {
        var timeOfFlight = timeOfFlightMap.get(robotState.getDistanceToHub().in(Meters));

        // Adjust the hub position by the negative of turret velocity during time of flight.
        // The turret's field-frame velocity includes the robot's translational velocity plus
        // the tangential velocity from rotation: v_turret = v_robot + omega x r_turret_field.
        var hubPose = robotState.getHubPose();
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

        var adjustedHubTranslation = new Translation2d(
                hubPose.getX() - turretVelocityX * timeOfFlight, hubPose.getY() - turretVelocityY * timeOfFlight);

        var currentTurretTranslation = robotState.getTurretPose().toPose2d().getTranslation();
        var turretDistanceFromHub =
                adjustedHubTranslation.minus(currentTurretTranslation).getNorm();

        var turretSetpoint = getTurretSetpointForAdjustedHub(adjustedHubTranslation);
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

    private TurretSetpoint getTurretSetpointForAdjustedHub(Translation2d adjustedHubTranslation) {
        var robotPoseNextLoop = PhysicsUtil.predictPoseFromFieldSpeeds(
                robotState.getPose(), fieldVelocity.get(), Constants.mainLoopPeriod);

        var currentTurretTranslation = robotState.getTurretPose().toPose2d().getTranslation();
        var turretHeading =
                adjustedHubTranslation.minus(currentTurretTranslation).getAngle();
        var predictedTargetTurretPosition = turretHeading.minus(robotPoseNextLoop.getRotation());

        return new TurretSetpoint(
                Radians.of(predictedTargetTurretPosition.getRadians()),
                RadiansPerSecond.of(-fieldVelocity.get().omegaRadiansPerSecond));
    }
}
