// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.turret.TurretConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagConfig;
import frc.robot.util.tuning.LoggedTunableMeasure;

import static edu.wpi.first.units.Units.*;

public final class VisionConstants {
    public static final double ambiguityThreshold = 0.4;
    public static final double fieldBorderMargin = 0.0;
    public static final double zMargin = 0.75;
    public static final boolean allowSingleTargetEstimates = false;

    public static final LoggedTunableMeasure<LinearVelocityUnit, LinearVelocity> maxRobotLinearVelocity =
            new LoggedTunableMeasure<LinearVelocityUnit, LinearVelocity>(
                    "Vision/MaxRobotLinearVelocity", MetersPerSecond.of(2.0));
    public static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> maxRobotAngularVelocity =
            new LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity>(
                    "Vision/MaxRobotAngularVelocity", RotationsPerSecond.of(0.2));
    public static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> maxTurretAngularVelocity =
            new LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity>(
                    "Vision/MaxTurretAngularVelocity", RotationsPerSecond.of(0.2));

    public static final double linearStdDevBaseline = 0.01; // Meters
    public static final double angularStdDevBaseline = 0.001; // Radians

    public static final SingleTargetMode singleTargetMode = SingleTargetMode.LOWEST_AMBIGUITY;

    public static enum SingleTargetMode {
        LOWEST_AMBIGUITY,
        CLOSEST_TO_HEADING,
    }

    public static AprilTagConfig frontAprilTagConfig = AprilTagConfig.onTurret(
            "Turret",
            TurretConstants.robotToTurret,
            new Transform3d(
                    Inches.of(5),
                    Inches.zero(),
                    Inches.of(6.75),
                    new Rotation3d(Degrees.zero(), Degrees.of(-30), Degrees.zero())),
            (timestamp) -> RobotState.getInstance().getTurretRotation(timestamp));
}
