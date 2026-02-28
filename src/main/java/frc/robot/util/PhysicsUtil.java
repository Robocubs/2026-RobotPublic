// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.*;

public class PhysicsUtil {
    public static final double maxLatencySeconds = 0.1;

    public static <M extends Measure<U>, U extends Unit, VM extends Measure<VU>, VU extends PerUnit<U, TimeUnit>>
            M predictCurrentValue(M value, VM slope, double timestamp) {
        var latency = Math.min(Timer.getTimestamp() - timestamp, maxLatencySeconds);
        return predictValue(value, slope, Seconds.of(latency));
    }

    public static <M extends Measure<U>, U extends Unit, VM extends Measure<VU>, VU extends PerUnit<U, TimeUnit>>
            M predictFutureValue(M value, VM slope, double timestamp) {
        return predictValue(value, slope, Seconds.of(timestamp - Timer.getTimestamp()));
    }

    @SuppressWarnings("unchecked")
    public static <M extends Measure<U>, U extends Unit, VM extends Measure<VU>, VU extends PerUnit<U, TimeUnit>>
            M predictValue(M value, VM slope, Time delta) {
        return (M) value.plus((M) slope.times(delta));
    }

    public static Pose2d predictCurrentPoseFromRobotSpeeds(Pose2d pose, ChassisSpeeds robotSpeeds, double timestamp) {
        var latency = Math.min(Timer.getTimestamp() - timestamp, maxLatencySeconds);
        return predictPoseFromRobotSpeeds(pose, robotSpeeds, Seconds.of(latency));
    }

    public static Pose2d predictFuturePoseFromRobotSpeeds(Pose2d pose, ChassisSpeeds robotSpeeds, double timestamp) {
        return predictPoseFromRobotSpeeds(pose, robotSpeeds, Seconds.of(timestamp - Timer.getTimestamp()));
    }

    public static Pose2d predictPoseFromRobotSpeeds(Pose2d pose, ChassisSpeeds robotSpeeds, Time delta) {
        // Assumes robot speeds are constant and so the robot moves in an arc
        var deltaPose = robotSpeeds.times(delta.in(Seconds));
        return pose.exp(
                new Twist2d(deltaPose.vxMetersPerSecond, deltaPose.vyMetersPerSecond, deltaPose.omegaRadiansPerSecond));
    }

    public static Pose2d predictCurrentPoseFromFieldSpeeds(Pose2d pose, ChassisSpeeds robotSpeeds, double timestamp) {
        var latency = Math.min(Timer.getTimestamp() - timestamp, maxLatencySeconds);
        return predictPoseFromFieldSpeeds(pose, robotSpeeds, Seconds.of(latency));
    }

    public static Pose2d predictFuturePoseFromFieldSpeeds(Pose2d pose, ChassisSpeeds robotSpeeds, double timestamp) {
        return predictPoseFromFieldSpeeds(pose, robotSpeeds, Seconds.of(timestamp - Timer.getTimestamp()));
    }

    public static Pose2d predictPoseFromFieldSpeeds(Pose2d pose, ChassisSpeeds robotSpeeds, Time delta) {
        // Assumes field speeds are constant and so the robot moves in a straight line
        return new Pose2d(
                pose.getX() + robotSpeeds.vxMetersPerSecond * delta.in(Seconds),
                pose.getY() + robotSpeeds.vyMetersPerSecond * delta.in(Seconds),
                pose.getRotation().plus(new Rotation2d(robotSpeeds.omegaRadiansPerSecond * delta.in(Seconds))));
    }
}
