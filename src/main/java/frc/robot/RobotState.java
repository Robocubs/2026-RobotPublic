// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.turret.TurretConstants;
import frc.robot.util.GeometryUtil;
import frc.robot.util.PhysicsUtil;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

import static edu.wpi.first.units.Units.*;

public class RobotState {
    public static final double lookbackTimeSeconds = 1.0;

    private final Field2d field;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(lookbackTimeSeconds);
    private final TimeInterpolatableBuffer<Double> robotLinearVelocityMetersPerSecondBuffer =
            TimeInterpolatableBuffer.createDoubleBuffer(lookbackTimeSeconds);
    private final TimeInterpolatableBuffer<Double> robotAngularVelocityRadiansPerSecondBuffer =
            TimeInterpolatableBuffer.createDoubleBuffer(lookbackTimeSeconds);
    private final TimeInterpolatableBuffer<Rotation2d> turretRotationBuffer =
            TimeInterpolatableBuffer.createBuffer(lookbackTimeSeconds);
    private final TimeInterpolatableBuffer<Double> turretAngularVelocityRadiansPerSecondBuffer =
            TimeInterpolatableBuffer.createDoubleBuffer(lookbackTimeSeconds);

    private final @Getter MatchState matchState = new MatchState();

    private @Setter Consumer<VisionMeasurement[]> visionMeasurementConsumer = measurements -> {};
    private @Setter Consumer<Pose2d> poseResetConsumer = pose -> {};
    private @Setter Consumer<Rotation2d> headingResetConsumer = heading -> {};

    private @AutoLogOutput @Getter Pose2d pose = Pose2d.kZero;
    private @AutoLogOutput @Getter Pose3d turretPose = GeometryUtil.toPose3d(TurretConstants.robotToTurret);
    private @AutoLogOutput @Getter ChassisSpeeds robotVelocity = new ChassisSpeeds();
    private @AutoLogOutput @Getter ChassisSpeeds fieldVelocity = new ChassisSpeeds();
    private @AutoLogOutput @Getter ChassisSpeeds targetRobotVelocity = new ChassisSpeeds();
    private @AutoLogOutput @Getter ChassisSpeeds targetFieldVelocity = new ChassisSpeeds();
    private @AutoLogOutput @Getter LinearVelocity robotSpeed = MetersPerSecond.zero();
    private @AutoLogOutput @Getter LinearVelocity maxSpeed = DriveConstants.maxSpeed;

    @Getter(lazy = true)
    private static final RobotState instance = new RobotState();

    private RobotState() {
        field = new Field2d();

        SmartDashboard.putData("Field", field);
        SmartDashboard.putBoolean("IsSimulation", Robot.isSimulation());
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    public static boolean isRed() {
        return !isBlue();
    }

    void update() {
        matchState.update();
    }

    @AutoLogOutput
    public Pose3d getPose3d() {
        return new Pose3d(pose);
    }

    public Optional<Pose2d> getPose(double timestamp) {
        return poseBuffer.getSample(timestamp);
    }

    @AutoLogOutput
    public Rotation2d getHeading() {
        return pose.getRotation();
    }

    public Rotation2d getHeadingTo(Translation2d translation) {
        return translation.minus(pose.getTranslation()).getAngle();
    }

    public Rotation2d getHeadingTo(Pose2d pose) {
        return pose.getTranslation().minus(this.pose.getTranslation()).getAngle();
    }

    public Distance getDistanceTo(Translation2d translation) {
        return Meters.of(pose.getTranslation().getDistance(translation));
    }

    public Distance getDistanceTo(Pose2d pose) {
        return getDistanceTo(pose.getTranslation());
    }

    public LinearVelocity getRobotSpeed(double timestamp) {
        return MetersPerSecond.of(
                robotLinearVelocityMetersPerSecondBuffer.getSample(timestamp).orElse(0.0));
    }

    public AngularVelocity getRobotAngularVelocity(double timestamp) {
        return RadiansPerSecond.of(
                robotAngularVelocityRadiansPerSecondBuffer.getSample(timestamp).orElse(0.0));
    }

    public Rotation2d getTurretRotation() {
        return getTurretRotation(Timer.getTimestamp());
    }

    public Rotation2d getTurretRotation(double timestamp) {
        return turretRotationBuffer.getSample(timestamp).orElse(Rotation2d.kZero);
    }

    public AngularVelocity getTurretAngularVelocity() {
        return getTurretAngularVelocity(Timer.getTimestamp());
    }

    public AngularVelocity getTurretAngularVelocity(double timestamp) {
        return RadiansPerSecond.of(
                turretAngularVelocityRadiansPerSecondBuffer.getSample(timestamp).orElse(0.0));
    }

    public Rotation2d getTurretRotationTo(Pose2d pose) {
        return getTurretHeadingTo(pose).minus(getHeading());
    }

    public Rotation2d getTurretRotationTo(Translation2d translation) {
        return getTurretHeadingTo(translation).minus(getHeading());
    }

    public Rotation2d getTurretHeadingTo(Pose2d pose) {
        return getTurretHeadingTo(pose.getTranslation());
    }

    public Rotation2d getTurretHeadingTo(Translation2d translation) {
        return translation.minus(turretPose.toPose2d().getTranslation()).getAngle();
    }

    public Distance getTurretDistanceFrom(Pose2d pose) {
        return getTurretDistanceFrom(pose.getTranslation());
    }

    public Distance getTurretDistanceFrom(Translation2d translation) {
        return Meters.of(
                translation.minus(turretPose.toPose2d().getTranslation()).getNorm());
    }

    @AutoLogOutput
    public Distance getTurretDistanceFromHub() {
        return getDistanceToHub(getTurretPose().toPose2d());
    }

    @AutoLogOutput
    public Distance getDistanceToHub() {
        return getDistanceToHub(pose);
    }

    public Distance getDistanceToHub(Pose2d pose) {
        return Meters.of(
                getHubPose().getTranslation().minus(pose.getTranslation()).getNorm());
    }

    public Pose2d getHubPose() {
        return isBlue() ? FieldConstants.blueHubPose : FieldConstants.redHubPose;
    }

    public Rotation2d getTurretShootingRotation() {
        return getTurretShootingHeading().minus(getHeading());
    }

    @AutoLogOutput
    public Rotation2d getTurretShootingHeading() {
        return getTurretHeadingTo(getHubPose());
    }

    public void addDriveMeasurements(DriveMeasurement... measurements) {
        if (measurements.length == 0) {
            return;
        }

        for (var measurement : measurements) {
            poseBuffer.addSample(measurement.timestampSeconds, measurement.pose);
            robotSpeed =
                    MetersPerSecond.of(Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond));
            robotLinearVelocityMetersPerSecondBuffer.addSample(
                    measurement.timestampSeconds, robotSpeed.in(MetersPerSecond));
            robotAngularVelocityRadiansPerSecondBuffer.addSample(
                    measurement.timestampSeconds, robotVelocity.omegaRadiansPerSecond);
        }

        var lastMeasurement = measurements[measurements.length - 1];
        pose = PhysicsUtil.predictCurrentPoseFromRobotSpeeds(
                lastMeasurement.pose, lastMeasurement.measuredSpeeds, lastMeasurement.timestampSeconds);
        robotVelocity = lastMeasurement.measuredSpeeds;
        fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getHeading());
        targetRobotVelocity = lastMeasurement.targetSpeeds;
        targetFieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(targetRobotVelocity, getHeading());
        field.setRobotPose(pose);
    }

    public void addVisionMeasurements(VisionMeasurement... measurements) {
        visionMeasurementConsumer.accept(measurements);
    }

    public void addSuperstructureMeasurement(Angle turretRotation, AngularVelocity turretAngularVelocity) {
        turretRotationBuffer.addSample(Timer.getTimestamp(), new Rotation2d(turretRotation.in(Radians)));
        turretAngularVelocityRadiansPerSecondBuffer.addSample(
                Timer.getTimestamp(), turretAngularVelocity.in(RadiansPerSecond));
        turretPose = calculateTurretPose(pose, new Rotation2d(turretRotation));
    }

    public void addVisionMeasurementConsumer(Consumer<VisionMeasurement[]> consumer) {
        visionMeasurementConsumer = consumer;
    }

    public void resetPose(Pose2d pose) {
        this.pose = pose;
        turretPose = calculateTurretPose(pose, getTurretRotation());
        field.setRobotPose(pose);

        poseBuffer.clear();
        poseBuffer.addSample(Timer.getTimestamp(), pose);

        poseResetConsumer.accept(pose);
    }

    public void resetHeading(Rotation2d heading) {
        pose = new Pose2d(pose.getTranslation(), heading);
        turretPose = calculateTurretPose(pose, getTurretRotation());
        field.setRobotPose(pose);

        poseBuffer.clear();
        poseBuffer.addSample(Timer.getTimestamp(), new Pose2d(pose.getTranslation(), heading));

        headingResetConsumer.accept(heading);
    }

    public void setPath(Pose2d... poses) {
        field.getObject("Path").setPoses(poses);
    }

    private Pose3d calculateTurretPose(Pose2d robotPose, Rotation2d turretRotation) {
        return new Pose3d(robotPose)
                .plus(TurretConstants.robotToTurret)
                .plus(new Transform3d(Translation3d.kZero, new Rotation3d(turretRotation)));
    }

    @AllArgsConstructor
    public static class DriveMeasurement {
        public final double timestampSeconds;
        public final Pose2d pose;
        public final ChassisSpeeds measuredSpeeds;
        public final ChassisSpeeds targetSpeeds;
    }

    @AllArgsConstructor
    public static class VisionMeasurement {
        public final double timestampSeconds;
        public final Pose3d pose;
        public final Matrix<N3, N1> stdDevs;
    }
}
