// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision.apriltag.proto;

import java.util.Optional;

import com.robocubs.proto.Apriltag.ProtobufAprilTagPipelineResult;
import edu.wpi.first.util.protobuf.Protobuf;
import frc.robot.subsystems.vision.apriltag.AprilTagMultiTargetResult;
import frc.robot.subsystems.vision.apriltag.AprilTagPipelineResult;
import frc.robot.subsystems.vision.apriltag.AprilTagTarget;
import us.hebi.quickbuf.Descriptors.Descriptor;
import us.hebi.quickbuf.RepeatedMessage;

public class AprilTagPipelineResultProto implements Protobuf<AprilTagPipelineResult, ProtobufAprilTagPipelineResult> {
    @Override
    public Class<AprilTagPipelineResult> getTypeClass() {
        return AprilTagPipelineResult.class;
    }

    @Override
    public Descriptor getDescriptor() {
        return ProtobufAprilTagPipelineResult.getDescriptor();
    }

    @Override
    public ProtobufAprilTagPipelineResult createMessage() {
        return ProtobufAprilTagPipelineResult.newInstance();
    }

    @Override
    public AprilTagPipelineResult unpack(ProtobufAprilTagPipelineResult msg) {
        return new AprilTagPipelineResult(
                msg.getLatency(),
                msg.getTimestamp(),
                AprilTagTarget.proto.unpack(msg.getTargets()),
                msg.hasMultiTargetResult()
                        ? Optional.of(AprilTagMultiTargetResult.proto.unpack(msg.getMultiTargetResult()))
                        : Optional.empty());
    }

    public AprilTagPipelineResult[] unpack(RepeatedMessage<ProtobufAprilTagPipelineResult> msg) {
        var results = new AprilTagPipelineResult[msg.length()];
        for (int i = 0; i < results.length; i++) {
            results[i] = unpack(msg.get(i));
        }

        return results;
    }

    @Override
    public void pack(ProtobufAprilTagPipelineResult msg, AprilTagPipelineResult value) {
        msg.setLatency(value.latencyMilliseconds);
        msg.setTimestamp(value.timestamp);
        AprilTagTarget.proto.pack(msg.getMutableTargets(), value.targets);
        value.multiTargetResult.ifPresent(
                result -> AprilTagMultiTargetResult.proto.pack(msg.getMutableMultiTargetResult(), result));
    }

    public void pack(RepeatedMessage<ProtobufAprilTagPipelineResult> msg, AprilTagPipelineResult[] values) {
        var targets = msg.reserve(values.length);
        for (var target : values) {
            pack(targets.next(), target);
        }
    }

    @Override
    public boolean isImmutable() {
        return true;
    }
}
