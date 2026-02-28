// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.characterization;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.LoggedAutoChooser;
import lombok.Builder;
import lombok.Builder.Default;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor(access = lombok.AccessLevel.PRIVATE)
@Builder
public class SysIdCommandSet {
    private final SysIdRoutine sysIdRoutine;
    private final @Default BooleanSupplier untilForward = () -> false;
    private final @Default BooleanSupplier untilReverse = () -> false;
    private final @Default Supplier<Command> beforeStartingForward = () -> Commands.runOnce(() -> {});
    private final @Default Supplier<Command> beforeStartingReverse = () -> Commands.runOnce(() -> {});
    private final @Default Supplier<Command> alongWith = () -> Commands.runOnce(() -> {});
    private final @Default Runnable finallyDo = () -> {};

    public void addAutos(LoggedAutoChooser autoChooser, String name) {
        autoChooser.addCmd(name + " Quasistatic Forward", () -> sysIdRoutine
                .quasistatic(SysIdRoutine.Direction.kForward)
                .until(untilForward)
                .beforeStarting(beforeStartingForward.get())
                .alongWith(alongWith.get())
                .finallyDo(finallyDo));
        autoChooser.addCmd(name + " Quasistatic Reverse", () -> sysIdRoutine
                .quasistatic(SysIdRoutine.Direction.kReverse)
                .until(untilReverse)
                .beforeStarting(beforeStartingReverse.get())
                .alongWith(alongWith.get())
                .finallyDo(finallyDo));
        autoChooser.addCmd(name + " Dynamic Forward", () -> sysIdRoutine
                .dynamic(SysIdRoutine.Direction.kForward)
                .until(untilForward)
                .beforeStarting(beforeStartingForward.get())
                .alongWith(alongWith.get())
                .finallyDo(finallyDo));
        autoChooser.addCmd(name + " Dynamic Reverse", () -> sysIdRoutine
                .dynamic(SysIdRoutine.Direction.kReverse)
                .until(untilReverse)
                .beforeStarting(beforeStartingReverse.get())
                .alongWith(alongWith.get())
                .finallyDo(finallyDo));
    }

    @SuppressWarnings("unused")
    public static class SysIdCommandSetBuilder {
        private BooleanSupplier finishedForward;
        private BooleanSupplier finishedReverse;
        private Supplier<Command> beforeStartingForward;
        private Supplier<Command> beforeStartingReverse;

        public SysIdCommandSetBuilder finished(BooleanSupplier finished) {
            this.finishedForward = finished;
            this.finishedReverse = finished;
            return this;
        }

        public SysIdCommandSetBuilder beforeStarting(Supplier<Command> commandSupplier) {
            this.beforeStartingForward = commandSupplier;
            this.beforeStartingReverse = commandSupplier;
            return this;
        }
    }
}
