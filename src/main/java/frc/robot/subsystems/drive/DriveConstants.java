// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.UnitsUtil;
import frc.robot.util.tuning.LoggedTunableMeasure;
import frc.robot.util.tuning.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

public final class DriveConstants {
    public static final LoggedTunableNumber rotationP = new LoggedTunableNumber("Drive/RotationP", 7.0);
    public static final LoggedTunableNumber rotationD = new LoggedTunableNumber("Drive/RotationD", 0.04);

    public static final Distance driveBaseLength =
            Meters.of(TunerConstants.FrontLeft.LocationX - TunerConstants.BackLeft.LocationX);
    public static final Distance driveBaseWidth =
            Meters.of(TunerConstants.FrontLeft.LocationY - TunerConstants.FrontRight.LocationY);
    public static final Distance driveBaseRadius =
            Meters.of(Math.hypot(driveBaseLength.in(Meters), driveBaseWidth.in(Meters)) / 2.0);

    public static final Distance wheelRadius = Meters.of(TunerConstants.FrontLeft.WheelRadius);
    public static final double wheelCOF = 1.8;

    public static final Double driveMotorGearRatio = TunerConstants.FrontLeft.DriveMotorGearRatio;
    public static final Double steerMotorGearRatio = TunerConstants.FrontLeft.SteerMotorGearRatio;
    public static final Voltage driveFrictionVoltage = Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage);
    public static final Voltage steerFrictionVoltage = Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage);
    public static final MomentOfInertia steerInertia = KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia);

    public static final SwerveDrivetrainConstants drivetrainConstants = TunerConstants.DrivetrainConstants;
    public static final SwerveModuleConstants<?, ?, ?>[] moduleConstants = new SwerveModuleConstants[] {
        TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight,
    };

    public static final Translation2d[] moduleTranslations = java.util.Arrays.stream(moduleConstants)
            .map(module -> new Translation2d(module.LocationX, module.LocationY))
            .toArray(Translation2d[]::new);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    public static final Matrix<N3, N1> odometryStdDevs = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
    public static final Matrix<N3, N1> visionStdDevs = new Matrix<>(VecBuilder.fill(1, 1, 1));

    public static final Torque maxTorque = NewtonMeters.of(DCMotor.getKrakenX60Foc(1).KtNMPerAmp
            * TunerConstants.FrontLeft.SlipCurrent
            * 4
            * driveMotorGearRatio); // t = I * kt * numMotors * reduction
    public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.0);
    public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(
            maxTorque.in(NewtonMeters) / wheelRadius.in(Meters) / RobotConstants.robotMass.in(Kilograms));

    public static final AngularVelocity maxAngularVelocity = UnitsUtil.toAngularVelocity(maxSpeed, driveBaseRadius);
    public static final AngularAcceleration maxAngularAcceleration = maxAngularVelocity.div(Seconds.of(0.33));

    public static final double translationDeadband = 0.1;
    public static final double rotationDeadband = 0.1;

    public static final LoggedTunableMeasure<LinearVelocityUnit, LinearVelocity> maxSmoothShootingSpeed =
            new LoggedTunableMeasure<>("Drive/MaxSmoothShootingSpeed", MetersPerSecond.of(2.0));
    public static final LoggedTunableMeasure<LinearAccelerationUnit, LinearAcceleration> maxSmoothShootingAcceleration =
            new LoggedTunableMeasure<>("Drive/MaxSmoothShootingAcceleration", MetersPerSecondPerSecond.of(12.0));

    public static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> maxSmoothShootingAngularVelocity =
            new LoggedTunableMeasure<>("Drive/MaxSmoothShootingAngularVelocity", RadiansPerSecond.of(Math.PI * 1.0));
    public static final LoggedTunableMeasure<AngularAccelerationUnit, AngularAcceleration>
            maxSmoothShootingAngularAcceleration = new LoggedTunableMeasure<>(
                    "Drive/MaxSmoothShootingAngularAcceleration", RadiansPerSecondPerSecond.of(Math.PI * 8.0));

    public static final LoggedTunableMeasure<LinearVelocityUnit, LinearVelocity> maxSmoothPassingSpeed =
            new LoggedTunableMeasure<>("Drive/MaxSmoothPassingSpeed", MetersPerSecond.of(3.0));
    public static final LoggedTunableMeasure<LinearAccelerationUnit, LinearAcceleration> maxSmoothPassingAcceleration =
            new LoggedTunableMeasure<>("Drive/MaxSmoothPassingAcceleration", MetersPerSecondPerSecond.of(18.0));

    public static final LoggedTunableMeasure<AngularVelocityUnit, AngularVelocity> maxSmoothPassingAngularVelocity =
            new LoggedTunableMeasure<>("Drive/MaxSmoothPassingAngularVelocity", RadiansPerSecond.of(Math.PI * 1.0));
    public static final LoggedTunableMeasure<AngularAccelerationUnit, AngularAcceleration>
            maxSmoothPassingAngularAcceleration = new LoggedTunableMeasure<>(
                    "Drive/MaxSmoothPassingAngularAcceleration", RadiansPerSecondPerSecond.of(Math.PI * 8.0));
}
