// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import java.util.stream.IntStream;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.*;

import static edu.wpi.first.units.Units.*;

public final class FieldConstants {
    public static final AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Distance fieldLength = Meters.of(fieldLayout.getFieldLength());
    public static final Distance fieldWidth = Meters.of(fieldLayout.getFieldWidth());
    public static final Pose2d origin = GeometryUtil.autoFlip(Pose2d.kZero);
    public static final Translation2d center = new Translation2d(fieldLength.div(2.0), fieldWidth.div(2.0));

    public static final int[] blueHubPoses = new int[] {18, 19, 20, 21, 24, 25, 26, 27};
    public static final int[] redHubPoses = new int[] {2, 3, 4, 5, 8, 9, 10, 11};
    public static final int[] allHubPoses = IntStream.concat(IntStream.of(blueHubPoses), IntStream.of(redHubPoses))
            .toArray();

    public static final double redAllianceThreshold =
            fieldLayout.getTagPose(10).get().getX();
    public static final double blueAllianceThreshold =
            fieldLayout.getTagPose(26).get().getX();
    public static final double redNeutralZoneStart =
            fieldLayout.getTagPose(4).get().getX();
    public static final double blueNeutralZoneStart =
            fieldLayout.getTagPose(20).get().getX();
    public static final double blueHubLeftThresholdY =
            fieldLayout.getTagPose(21).get().getY();
    public static final double blueHubRightThresholdY =
            fieldLayout.getTagPose(18).get().getY();
    public static final double redHubLeftThresholdY =
            fieldLayout.getTagPose(5).get().getY();
    public static final double redHubRightThresholdY =
            fieldLayout.getTagPose(2).get().getY();

    public static final Pose2d redHubPose = fieldLayout
            .getTagPose(10)
            .get()
            .toPose2d()
            .transformBy(new Transform2d(Inches.of(-23.5), Inches.zero(), Rotation2d.kZero));
    public static final Pose2d blueHubPose = fieldLayout
            .getTagPose(26)
            .get()
            .toPose2d()
            .transformBy(new Transform2d(Inches.of(-23.5), Inches.zero(), Rotation2d.kZero));

    public static final Pose2d redTowerPose = fieldLayout
            .getTagPose(15)
            .get()
            .toPose2d()
            .transformBy(new Transform2d(Inches.of(43), Inches.zero(), Rotation2d.kZero));
    public static final Pose2d blueTowerPose = fieldLayout
            .getTagPose(31)
            .get()
            .toPose2d()
            .transformBy(new Transform2d(Inches.of(43), Inches.zero(), Rotation2d.kZero));
}
