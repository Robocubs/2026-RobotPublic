// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureSetpoint;
import frc.robot.subsystems.superstructure.turret.Turret.TurretSetpoint;
import frc.robot.subsystems.superstructure.turret.TurretConstants;
import frc.robot.util.PhysicsUtil;

import static edu.wpi.first.units.Units.*;

public class SuperstructureSetpointCalculator {
    private static final InterpolatingDoubleTreeMap flywheelMap = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.17, 250.0),
            Map.entry(1.85, 260.0),
            Map.entry(2.75, 270.0),
            Map.entry(3.69, 285.0),
            Map.entry(4.46, 310.0),
            Map.entry(5.20, 330.0),
            Map.entry(6.07, 350.0));
    private static final InterpolatingDoubleTreeMap hoodMap = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.17, 0.2),
            Map.entry(1.85, 0.4),
            Map.entry(2.75, 0.5),
            Map.entry(3.69, 0.6),
            Map.entry(4.46, 0.7),
            Map.entry(5.20, 0.75),
            Map.entry(6.07, 0.8));
    // private static final InterpolatingDoubleTreeMap timeOfFlightMap = InterpolatingDoubleTreeMap.ofEntries(
    //         Map.entry(1.17, 1.5),
    //         Map.entry(1.85, 1.5),
    //         Map.entry(2.75, 1.5),
    //         Map.entry(3.69, 1.5),
    //         Map.entry(4.46, 1.5),
    //         Map.entry(5.20, 1.5),
    //         Map.entry(6.07, 1.5));

    private final RobotState robotState = RobotState.getInstance();

    public SuperstructureSetpoint getSetpointForHub() {
        var turretSetpoint = getPredictedTurretSetpointForHub();
        var turretDistanceFromHub = robotState.getTurretDistanceFromHub().in(Meters);
        var flywheelVelocity = flywheelMap.get(turretDistanceFromHub);
        var hoodAngle = hoodMap.get(turretDistanceFromHub);

        return new SuperstructureSetpoint(
                RadiansPerSecond.of(flywheelVelocity),
                Radians.of(hoodAngle),
                turretSetpoint.position(),
                turretSetpoint.velocity());
    }

    private TurretSetpoint getPredictedTurretSetpointForHub() {
        var predictedRobotPose = PhysicsUtil.predictPoseFromFieldSpeeds(
                robotState.getPose(), robotState.getTargetFieldVelocity(), Constants.mainLoopPeriod);
        var predictedTurretPose = predictedRobotPose.transformBy(TurretConstants.robotToTurret2d);
        var hubPose = robotState.getHubPose();
        var predictedTurretHeading = hubPose.getTranslation()
                .minus(predictedTurretPose.getTranslation())
                .getAngle();
        var predictedTargetTurretPosition = predictedTurretHeading.minus(predictedRobotPose.getRotation());
        var currentTargetTurretPosition = robotState.getTurretRotationTo(hubPose);
        var velocityTarget = MathUtil.angleModulus(
                        predictedTargetTurretPosition.getRadians() - currentTargetTurretPosition.getRadians())
                / Constants.mainLoopPeriod.in(Seconds);
        return new TurretSetpoint(
                Radians.of(predictedTargetTurretPosition.getRadians()), RadiansPerSecond.of(velocityTarget));
    }
}
