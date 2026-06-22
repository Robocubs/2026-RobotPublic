// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;

public class PhoenixConfigurationThread implements AutoCloseable {
    private static final int MAX_RETRIES = 5;

    private final BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private final ThreadPoolExecutor executor = new ThreadPoolExecutor(1, 1, 5, TimeUnit.MILLISECONDS, queue);

    public CompletableFuture<Void> tryUntilOk(Supplier<StatusCode> action) {
        return CompletableFuture.runAsync(
                () -> {
                    var lastStatus = StatusCode.OK;
                    for (int i = 0; i < MAX_RETRIES; i++) {
                        lastStatus = action.get();
                        if (lastStatus.isOK()) {
                            return;
                        }
                    }

                    DriverStation.reportError("Phoenix configuration failed with status: " + lastStatus, null);
                },
                executor);
    }

    @Override
    public void close() {
        executor.shutdownNow();
    }
}
