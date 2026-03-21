// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.apriltag;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.*;

public class AprilTagCamera {
    private final AprilTagIO io;
    private final AprilTagIOInputsAutoLogged inputs = new AprilTagIOInputsAutoLogged();
    private final AprilTagConfig config;
    private final RobotState robotState = RobotState.getInstance();
    private final Alert disconnectedAlert;
    private final String loggingPrefix;

    public AprilTagCamera(AprilTagIO io) {
        this.io = io;
        config = io.getConfig();
        disconnectedAlert = new Alert("Vision camera " + config.cameraName + " is disconnected.", AlertType.kWarning);
        loggingPrefix = "Vision/Camera" + config.cameraName;
    }

    public void update(List<RobotState.VisionMeasurement> measurements) {
        io.updateInputs(inputs);
        Logger.processInputs(loggingPrefix, inputs);
        disconnectedAlert.set(!inputs.isConnected);

        List<Pose3d> targetPoses = new ArrayList<>(inputs.pipelineResults.results.length * 22);
        List<Pose3d> usedTargetPoses = new ArrayList<>(inputs.pipelineResults.results.length * 22);
        List<Pose3d> acceptedRobotPoses = new ArrayList<>(inputs.pipelineResults.results.length);
        List<Pose3d> rejectedRobotPoses = new ArrayList<>(inputs.pipelineResults.results.length);

        for (var pipelineResult : inputs.pipelineResults.results) {
            var robotPose =
                    new Pose3d(robotState.getPose(pipelineResult.timestamp).orElse(robotState.getPose()));
            var robotToCamera = config.robotToCamera.apply(pipelineResult.timestamp);
            var cameraToRobot = robotToCamera.inverse();

            Stream.of(pipelineResult.targets)
                    .map(target -> robotPose.plus(robotToCamera).plus(target.bestCameraToTarget))
                    .forEach(targetPoses::add);

            var robotAngularVelocity = robotState.getRobotAngularVelocity(pipelineResult.timestamp);
            var turretAngularVelocity =
                    robotAngularVelocity.plus(robotState.getTurretAngularVelocity(pipelineResult.timestamp));
            var isValidRobotState = config.acceptPoses
                    && robotState.getRobotSpeed(pipelineResult.timestamp).lt(maxRobotLinearVelocity.get())
                    && robotAngularVelocity.isNear(RadiansPerSecond.zero(), maxRobotAngularVelocity.get())
                    && turretAngularVelocity.isNear(RadiansPerSecond.zero(), maxTurretAngularVelocity.get());

            if (pipelineResult.multiTargetResult.isPresent()) {
                var multiTargetResult = pipelineResult.multiTargetResult.get();
                var observedPose = multiTargetResult
                        .cameraPose
                        .relativeTo(FieldConstants.fieldLayout.getOrigin())
                        .plus(cameraToRobot);

                if (isValidRobotState && isValidPose(observedPose)) {
                    acceptedRobotPoses.add(observedPose);
                    measurements.add(new RobotState.VisionMeasurement(
                            pipelineResult.timestamp,
                            observedPose,
                            getStdDevs(multiTargetResult.targetIds, multiTargetResult.cameraPose)));
                    IntStream.of(multiTargetResult.targetIds)
                            .mapToObj(FieldConstants.fieldLayout::getTagPose)
                            .filter(Optional::isPresent)
                            .map(Optional::get)
                            .forEach(usedTargetPoses::add);
                    continue;
                }

                rejectedRobotPoses.add(observedPose);
            }

            if (!allowSingleTargetEstimates) {
                continue;
            }

            if (!isValidRobotState || pipelineResult.targets.length == 0) {
                continue;
            }

            var lowestAmbiguityTarget = pipelineResult.targets[0];
            for (var i = 1; i < pipelineResult.targets.length; i++) {
                if (pipelineResult.targets[i].ambiguity < lowestAmbiguityTarget.ambiguity) {
                    lowestAmbiguityTarget = pipelineResult.targets[i];
                }
            }

            FieldConstants.fieldLayout.getTagPose(lowestAmbiguityTarget.id).ifPresent(usedTargetPoses::add);

            var targetPose = FieldConstants.fieldLayout.getTagPose(lowestAmbiguityTarget.id);
            if (targetPose.isEmpty()) {
                continue;
            }

            var observedPose = targetPose
                    .get()
                    .transformBy(lowestAmbiguityTarget.bestCameraToTarget.inverse())
                    .transformBy(cameraToRobot);

            if (!isValidPose(observedPose)) {
                continue;
            }

            acceptedRobotPoses.add(observedPose);
            measurements.add(new RobotState.VisionMeasurement(
                    pipelineResult.timestamp,
                    observedPose,
                    getStdDevs(
                            1,
                            lowestAmbiguityTarget
                                    .bestCameraToTarget
                                    .getTranslation()
                                    .getNorm())));
            usedTargetPoses.add(targetPose.get());
        }

        Logger.recordOutput(loggingPrefix + "/TargetPoses", targetPoses.toArray(Pose3d[]::new));
        Logger.recordOutput(loggingPrefix + "/UsedTargetPoses", usedTargetPoses.toArray(Pose3d[]::new));
        Logger.recordOutput(loggingPrefix + "/AcceptedPoses", acceptedRobotPoses.toArray(Pose3d[]::new));
        Logger.recordOutput(loggingPrefix + "/RejectedPoses", rejectedRobotPoses.toArray(Pose3d[]::new));
    }

    private boolean isValidPose(Pose3d pose) {
        return pose.getX() > fieldBorderMargin
                && pose.getY() > fieldBorderMargin
                && pose.getX() < FieldConstants.fieldLayout.getFieldLength() - fieldBorderMargin
                && pose.getY() < FieldConstants.fieldLayout.getFieldWidth() - fieldBorderMargin
                && pose.getZ() > -zMargin
                && pose.getZ() < zMargin;
    }

    private Vector<N3> getStdDevs(int[] targetIds, Pose3d cameraPose) {
        var distance = IntStream.of(targetIds)
                .boxed()
                .map(FieldConstants.fieldLayout::getTagPose)
                .filter(Optional::isPresent)
                .mapToDouble(targetPose -> targetPose.get().getTranslation().getDistance(cameraPose.getTranslation()))
                .average()
                .orElse(10);
        return getStdDevs(targetIds.length, distance);
    }

    private Vector<N3> getStdDevs(double numTargets, double distance) {
        var linear = linearStdDevBaseline * distance * distance * config.weight / numTargets;
        var rotation = numTargets > 1
                ? angularStdDevBaseline * distance * distance * config.weight / numTargets
                : angularStdDevBaseline * distance * distance * config.weight * 1000;
        return VecBuilder.fill(linear, linear, rotation);
    }
}
