// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.controllers;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class PathController {
    private static final double translationP = 10.0;
    private static final double translationI = 0.0;
    private static final double translationD = 0.0;
    private static final double rotationP = 7.0;
    private static final double rotationI = 0.0;
    private static final double rotationD = 0.0;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    public PathController() {
        xController = new PIDController(translationP, translationI, translationD);
        yController = new PIDController(translationP, translationI, translationD);
        rotationController = new PIDController(rotationP, rotationI, rotationD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ChassisSpeeds update(SwerveSample sample, Pose2d pose, LinearVelocity maxSpeed) {
        var targetPose = sample.getPose();

        var vx = sample.vx + xController.calculate(pose.getTranslation().getX(), targetPose.getX());
        var vy = sample.vy + yController.calculate(pose.getTranslation().getY(), targetPose.getY());
        var omega = sample.omega
                + rotationController.calculate(
                        pose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians());

        var speed = Math.hypot(vx, vy);
        if (speed > maxSpeed.in(MetersPerSecond)) {
            vx = maxSpeed.in(MetersPerSecond) / speed * vx;
            vy = maxSpeed.in(MetersPerSecond) / speed * vy;
        }

        Logger.recordOutput("Choreo/TargetPose", targetPose);

        return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, pose.getRotation());
    }
}
