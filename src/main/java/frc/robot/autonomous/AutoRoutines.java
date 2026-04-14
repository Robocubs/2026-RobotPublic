// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autonomous;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import frc.robot.commands.SubsystemScheduler;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.util.GeometryUtil;
import org.littletonrobotics.junction.Logger;

public class AutoRoutines {
    private static final Pose2d leftStartingPose = new Pose2d(3.45, 5.6, Rotation2d.fromDegrees(135));
    private static final Pose2d rightStartingPose = new Pose2d(3.45, 2.4, Rotation2d.fromDegrees(-135));

    private final RobotState robotState = RobotState.getInstance();
    private final Drive drive;
    private final AutoFactory factory;
    private final SubsystemScheduler<Superstructure> superstructure;
    private final SubsystemScheduler<Intake> intake;

    public AutoRoutines(
            Drive drive, SubsystemScheduler<Superstructure> superstructure, SubsystemScheduler<Intake> intake) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.intake = intake;

        factory = new AutoFactory(
                robotState::getPose, robotState::resetPose, drive::followPath, true, drive, (trajectory, starting) -> {
                    if (starting) {
                        var poses =
                                RobotState.isBlue() ? trajectory.getPoses() : GeometryUtil.flip(trajectory.getPoses());
                        Logger.recordOutput("Choreo/Trajectory", poses);
                    } else {
                        Logger.recordOutput("Choreo/Trajectory", new Pose2d[] {});
                        Logger.recordOutput("Choreo/TargetPose", Pose2d.kZero);
                    }
                });
    }

    private LoggedAutoRoutine create(String name) {
        return new LoggedAutoRoutine(name, factory, drive, superstructure, intake);
    }

    public AutoRoutine fastLeftWithDepot() {
        return create("FastLeftWithDepot")
                .requireStartingPose("FastLeft")
                .intake()
                .setSuperstructureState(SuperstructureState.TRACKING_HUB_FOR_POSE)
                .followPath("FastLeft", 0)
                .shoot()
                .followPath("FastLeft", 1)
                .agitateIntake()
                .driveToFinalPose("FastLeft", 1, false, true)
                .build();
    }

    public AutoRoutine fastLeft() {
        return create("FastLeft")
                .requireStartingPose("FastLeft")
                .intake()
                .setSuperstructureState(SuperstructureState.TRACKING_HUB_FOR_POSE)
                .followPath("FastLeft", 0)
                .shoot()
                .driveToFinalPose("FastLeft", 1, true, true)
                .agitateIntake()
                .driveToFinalPose("FastLeft", 1, false, true)
                .build();
    }

    public AutoRoutine newLeftFinalVersion() {
        return create("NewLeftFinalVersion")
                .requireStartingPose(leftStartingPose)
                .intake()
                .setSuperstructureState(SuperstructureState.TRACKING_HUB_FOR_POSE)
                .driveToInitialPose("NewLeftFinalVersion", true)
                .followPath("NewLeftFinalVersion")
                .agitateIntake()
                .shoot(12.4)
                .build();
    }

    public AutoRoutine newRightFinalVersion() {
        return create("NewRightFinalVersion")
                .requireStartingPose(rightStartingPose)
                .intake()
                .setSuperstructureState(SuperstructureState.TRACKING_HUB_FOR_POSE)
                .driveToInitialPose("NewRightFinalVersion", true)
                .followPath("NewRightFinalVersion")
                .agitateIntake()
                .shoot(12.4)
                .build();
    }

    public AutoRoutine stationToBump() {
        return create("StationToBump")
                .intakeEfficient()
                .shoot()
                .driveToStation(3)
                .followPath("StationToBump", 0)
                .agitateIntake()
                .shoot(15)
                .build();
    }

    public AutoRoutine shootInPlace() {
        return create("ShootInPlace").shoot(20).build();
    }
}
