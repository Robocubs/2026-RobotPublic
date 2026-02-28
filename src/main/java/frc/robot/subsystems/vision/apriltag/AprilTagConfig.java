// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.apriltag;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class AprilTagConfig {
    public final String cameraName;
    public final Function<Double, Transform3d> robotToCamera;
    public final double weight;

    public AprilTagConfig(String cameraName, Transform3d robotToCamera) {
        this.cameraName = cameraName;
        this.robotToCamera = (ignored) -> robotToCamera;
        this.weight = 1.0;
    }

    public AprilTagConfig(String cameraName, Function<Double, Transform3d> robotToCamera) {
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
        this.weight = 1.0;
    }

    public static AprilTagConfig onTurret(
            String cameraName,
            Transform3d robotToTurret,
            Transform3d turretToCamera,
            Function<Double, Rotation2d> turretRotation) {
        return new AprilTagConfig(cameraName, (timestamp) -> {
            var rotation = new Transform3d(Translation3d.kZero, new Rotation3d(turretRotation.apply(timestamp)));
            return robotToTurret.plus(rotation).plus(turretToCamera);
        });
    }
}
