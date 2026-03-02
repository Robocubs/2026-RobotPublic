// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.simulation;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.Consumer;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class SimNotifier {
    private static final double kSimLoopPeriod = 0.0004;

    private static SimNotifier instance = null;

    private final List<Consumer<Time>> callbacks = new CopyOnWriteArrayList<>();
    private final Notifier simNotifier;
    private long lastSimTime;

    private SimNotifier() {
        lastSimTime = RobotController.getFPGATime();
        simNotifier = new Notifier(() -> {
            try {
                var currentTime = RobotController.getFPGATime();
                var deltaTime = Microseconds.of(currentTime - lastSimTime);
                lastSimTime = currentTime;

                for (var callback : callbacks) {
                    callback.accept(deltaTime);
                }

                Logger.recordOutput("Simulation/LoopMilliseconds", deltaTime.in(Milliseconds));
            } catch (Exception e) {
                System.err.println("Error in simulation notifier callback");
                e.printStackTrace();
            }
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public static synchronized void register(Consumer<Time> callback) {
        if (instance == null) {
            instance = new SimNotifier();
        }

        instance.callbacks.add(callback);
    }
}
