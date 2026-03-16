// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;

import static edu.wpi.first.units.Units.*;

public class SmoothDriveController {

    private Translation2d previousVelocity = Translation2d.kZero;
    private double previousOmega = 0.0;
    private double lastLoopTimestamp = 0.0;
    private double loopDeltaTimeSeconds = 0.0;

    public void updateLoopTime() {
        var timestampSeconds = Timer.getTimestamp();
        loopDeltaTimeSeconds = timestampSeconds - lastLoopTimestamp;
        lastLoopTimestamp = timestampSeconds;
    }

    public void updateFromMeasuredValues() {
        var maxSpeed = DriveConstants.maxSmoothSpeed.get().in(MetersPerSecond);
        var maxOmega = DriveConstants.maxSmoothAngularVelocity.get().in(RadiansPerSecond);
        var speeds = RobotState.getInstance().getFieldVelocity();

        var speedsTranslation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        previousVelocity = speedsTranslation.getNorm() > maxSpeed
                ? speedsTranslation.times(maxSpeed / speedsTranslation.getNorm())
                : speedsTranslation;

        previousOmega = MathUtil.clamp(speeds.omegaRadiansPerSecond, -maxOmega, maxOmega);
    }

    public void resetFromMeasuredValues() {
        loopDeltaTimeSeconds = Constants.mainLoopPeriod.in(Seconds);
        lastLoopTimestamp = Timer.getTimestamp() - loopDeltaTimeSeconds;
        updateFromMeasuredValues();
    }

    public Translation2d smoothLinearVelocity(Translation2d targetVelocity) {
        var maxSpeed = DriveConstants.maxSmoothSpeed.get().in(MetersPerSecond);
        var maxAcceleration = DriveConstants.maxSmoothAcceleration.get().in(MetersPerSecondPerSecond);
        var velocityDeltaLimit = maxAcceleration * loopDeltaTimeSeconds;

        // Limit to max speed
        var newVelocity = targetVelocity.getNorm() > maxSpeed
                ? new Translation2d(maxSpeed, targetVelocity.getAngle())
                : targetVelocity;

        // Limit to max acceleration in 2D space
        var velocityDelta = newVelocity.minus(previousVelocity);
        var deltaNorm = velocityDelta.getNorm();
        if (deltaNorm > velocityDeltaLimit) {
            velocityDelta = velocityDelta.times(velocityDeltaLimit / deltaNorm);
            newVelocity = previousVelocity.plus(velocityDelta);
        }

        previousVelocity = newVelocity;
        return newVelocity;
    }

    public double smoothAngularVelocity(double targetOmega) {
        var maxOmega = DriveConstants.maxSmoothAngularVelocity.get().in(RadiansPerSecond);
        var maxAlpha = DriveConstants.maxSmoothAngularAcceleration.get().in(RadiansPerSecondPerSecond);
        var omegaDeltaLimit = maxAlpha * loopDeltaTimeSeconds;

        // Limit to max speed
        var newOmega = MathUtil.clamp(targetOmega, -maxOmega, maxOmega);

        // Limit to max acceleration
        var omegaDelta = newOmega - previousOmega;
        if (Math.abs(omegaDelta) > omegaDeltaLimit) {
            newOmega = previousOmega + Math.copySign(omegaDeltaLimit, omegaDelta);
        }

        previousOmega = newOmega;
        return newOmega;
    }
}
