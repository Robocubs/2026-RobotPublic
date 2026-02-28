// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.apriltag;

import java.util.Optional;

import edu.wpi.first.util.protobuf.ProtobufSerializable;
import frc.robot.subsystems.vision.apriltag.proto.AprilTagPipelineResultProto;

public class AprilTagPipelineResult implements ProtobufSerializable {
    public static final AprilTagPipelineResult kEmpty =
            new AprilTagPipelineResult(0, 0, new AprilTagTarget[] {}, Optional.empty());

    public final double latencyMilliseconds;
    public final double timestamp;
    public final AprilTagTarget[] targets;
    public final Optional<AprilTagMultiTargetResult> multiTargetResult;

    public AprilTagPipelineResult(
            double latencyMilliseconds,
            double timestamp,
            AprilTagTarget[] targets,
            Optional<AprilTagMultiTargetResult> multiTargetResult) {
        this.latencyMilliseconds = latencyMilliseconds;
        this.timestamp = timestamp;
        this.targets = targets;
        this.multiTargetResult = multiTargetResult;
    }

    public static final AprilTagPipelineResultProto proto = new AprilTagPipelineResultProto();
}
