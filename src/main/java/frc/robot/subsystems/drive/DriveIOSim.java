// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.RobotConstants;
import frc.robot.simulation.MapleSimSwerveDrivetrain;
import frc.robot.simulation.SimRobotState;

public class DriveIOSim extends DriveIOHardware {
    private final Notifier simNotifier;
    private final MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain;
    private double lastSimTime;

    public DriveIOSim(SimRobotState simRobotState) {
        lastSimTime = Utils.getCurrentTimeSeconds();

        if (Constants.mapleSimEnabled) {
            simNotifier = null;
            mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                    RobotConstants.robotMass,
                    RobotConstants.robotLengthWithBumpers,
                    RobotConstants.robotWidthWithBumpers,
                    DCMotor.getKrakenX60Foc(1),
                    DCMotor.getKrakenX44Foc(1),
                    MotorType.KrakenX60,
                    MotorType.KrakenX44,
                    DriveConstants.wheelCOF,
                    swerveDrivetrain.getModuleLocations(),
                    swerveDrivetrain.getPigeon2(),
                    swerveDrivetrain.getModules(),
                    DriveConstants.moduleConstants);

            simRobotState.init(mapleSimSwerveDrivetrain);
        } else {
            mapleSimSwerveDrivetrain = null;
            simNotifier = new Notifier(() -> {
                final var currentTime = Utils.getCurrentTimeSeconds();
                var deltaTime = currentTime - lastSimTime;
                lastSimTime = currentTime;
                swerveDrivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
            simNotifier.startPeriodic(SimRobotState.simLoopPeriod);
        }
    }

    @Override
    protected void regulateModuleConstants(SwerveModuleConstants<?, ?, ?>[] constants) {
        super.regulateModuleConstants(constants);
        if (Constants.mapleSimEnabled) {
            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(DriveConstants.moduleConstants);
        }
    }
}
