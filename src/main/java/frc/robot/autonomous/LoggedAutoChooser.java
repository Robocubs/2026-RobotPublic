// Copyright (c) 2026 FRC Team 1701 Robocubs
// http://github.com/Robocubs
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.autonomous;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.util.ChoreoAlert;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

/**
 * This is a forked version of the Choreo {@code AutoChooser} but as a {@code LoggedNetworkInput}.
 *
 * It allows for the selection of {@link AutoRoutine}s at runtime via a <a
 * href="https://docs.wpilib.org/en/stable/docs/software/dashboards/index.html#dashboards">Dashboard</a>.
 *
 * <p>This chooser takes a <a href="https://en.wikipedia.org/wiki/Lazy_loading">lazy loading</a>
 * approach to {@link AutoRoutine}s, only generating the {@link AutoRoutine} when it is selected.
 * This approach has the benefit of not loading all autos on startup, but also not loading the auto
 * during auto start causing a delay.
 *
 * <p>Once the {@link LoggedAutoChooser} is made you can add {@link LoggedAutoChooser}s to it using
 * {@link #addRoutine} or add {@link Command}s to it using {@link #addCmd}. Similar to {@code
 * SendableChooser} this chooser can be added to the {@link
 * edu.wpi.first.wpilibj.smartdashboard.SmartDashboard} using {@code
 * SmartDashboard.putData(Sendable)}.
 *
 * <p>You can set the Robot's autonomous command to the chooser's chosen auto routine via <code>
 * RobotModeTriggers.autonomous.whileTrue(chooser.autoSchedulingCmd());</code>
 */
public class LoggedAutoChooser extends LoggedNetworkInput implements Sendable {
    public static final String DO_NOTHING_NAME = "Nothing";
    private static final Alert selectedNonexistentAuto =
            ChoreoAlert.alert("Selected an auto that isn't an option", kError);

    private final String loggingKey;
    private final HashMap<String, Supplier<Command>> autoRoutines = new HashMap<>();

    private String selected;
    private String[] options = new String[] {};

    private Optional<Alliance> allianceAtGeneration = Optional.empty();
    private String nameAtGeneration;
    private Command generatedCommand = Commands.none();

    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(loggingKey, selected);
        }

        public void fromLog(LogTable table) {
            select(table.get(loggingKey, selected));
        }
    };

    /**
     * Constructs a new {@link LoggableInputs} with the given name for the logging key.
     *
     * @param key The key for the chooser, published to "/SmartDashboard/{key}" for NT or
     *     "/DashboardInputs/SmartDashboard/{key}" when logged.
     */
    public LoggedAutoChooser(String key) {
        loggingKey = key;

        nameAtGeneration = DO_NOTHING_NAME;
        generatedCommand = Commands.none();
        addCmd(DO_NOTHING_NAME, Commands::none);
        select(DO_NOTHING_NAME);

        SmartDashboard.putData(key, this);
        periodic();
        Logger.registerDashboardInput(this);
    }

    /**
     * Select a new option in the chooser.
     *
     * <p>This method is called automatically when published as a sendable.
     *
     * @param selectStr The name of the option to select.
     * @return The name of the selected option.
     */
    public String select(String selectStr) {
        return select(selectStr, false);
    }

    private String select(String selectStr, boolean force) {
        selected = selectStr;
        if (selected.equals(nameAtGeneration) && allianceAtGeneration.equals(DriverStation.getAlliance())) {
            // early return if the selected auto matches the active auto
            return nameAtGeneration;
        }
        boolean dsValid =
                DriverStation.isDisabled() && DriverStation.getAlliance().isPresent();
        if (dsValid || force) {
            if (!autoRoutines.containsKey(selected) && !selected.equals(DO_NOTHING_NAME)) {
                selected = DO_NOTHING_NAME;
                selectedNonexistentAuto.set(true);
            } else {
                selectedNonexistentAuto.set(false);
            }
            allianceAtGeneration = DriverStation.getAlliance();
            nameAtGeneration = selected;
            generatedCommand = autoRoutines.get(nameAtGeneration).get().withName(nameAtGeneration);
        } else {
            allianceAtGeneration = Optional.empty();
            nameAtGeneration = DO_NOTHING_NAME;
            generatedCommand = Commands.none();
        }
        return nameAtGeneration;
    }

    /**
     * Add an AutoRoutine to the chooser.
     *
     * <p>This is done to load AutoRoutines when and only when they are selected, in order to save
     * memory and file loading time for unused AutoRoutines.
     *
     * <p>The generators are only run when the DriverStation is disabled and the alliance is known.
     *
     * <p>One way to keep this clean is to make an `Autos` class that all of your subsystems/resources
     * are <a href="https://en.wikipedia.org/wiki/Dependency_injection">dependency injected</a> into.
     * Then create methods inside that class that take an {@link AutoFactory} and return an {@link
     * AutoRoutine}.
     *
     * <h3>Example:</h3>
     *
     * <pre><code>
     * LoggedAutoChooser chooser;
     * Autos autos = new Autos(swerve, shooter, intake, feeder);
     * public Robot() {
     *   chooser = new LoggedAutoChooser("/Choosers");
     *   SmartDashboard.putData(chooser);
     *   // fourPieceRight is a method that accepts an AutoFactory and returns an AutoRoutine.
     *   chooser.addRoutine("4 Piece right", autos::fourPieceRight);
     *   chooser.addRoutine("4 Piece Left", autos::fourPieceLeft);
     *   chooser.addRoutine("3 Piece Close", autos::threePieceClose);
     * }
     * </code></pre>
     *
     * @param name The name of the auto routine.
     * @param generator The function that generates the auto routine.
     * @return This {@link LoggedAutoChooser} instance, to allow for method chaining.
     */
    public LoggedAutoChooser addRoutine(String name, Supplier<AutoRoutine> generator) {
        autoRoutines.put(name, () -> generator.get().cmd());
        options = autoRoutines.keySet().stream().sorted().toArray(String[]::new);
        return this;
    }

    /**
     * Adds a Command to the auto chooser.
     *
     * <p>This is done to load autonomous commands when and only when they are selected, in order to
     * save memory and file loading time for unused autonomous commands.
     *
     * <p>The generators are only run when the DriverStation is disabled and the alliance is known.
     *
     * <h3>Example:</h3>
     *
     * <pre><code>
     * LoggedAutoChooser chooser;
     * Autos autos = new Autos(swerve, shooter, intake, feeder);
     * public Robot() {
     *   chooser = new LoggedAutoChooser("/Choosers");
     *   SmartDashboard.putData(chooser);
     *   // fourPieceLeft is a method that accepts an AutoFactory and returns a command.
     *   chooser.addCmd("4 Piece left", autos::fourPieceLeft);
     *   chooser.addCmd("Just Shoot", shooter::shoot);
     * }
     * </code></pre>
     *
     * @param name The name of the autonomous command.
     * @param generator The function that generates an autonomous command.
     * @return This {@link LoggedAutoChooser} instance, to allow for method chaining.
     * @see LoggedAutoChooser#addRoutine
     */
    public LoggedAutoChooser addCmd(String name, Supplier<Command> generator) {
        autoRoutines.put(name, generator);
        options = autoRoutines.keySet().stream().sorted().toArray(String[]::new);
        return this;
    }

    /**
     * Gets a Command that schedules the selected auto routine. This Command shares the lifetime of
     * the scheduled Command. This Command can directly be bound to a trigger, like so:
     *
     * <pre><code>
     *     LoggedAutoChooser chooser = ...;
     *
     *     public Robot() {
     *         RobotModeTriggers.autonomous().whileTrue(chooser.selectedCommandScheduler());
     *     }
     * </code></pre>
     *
     * @return A command that runs the selected {@link AutoRoutine}
     */
    public Command selectedCommandScheduler() {
        return Commands.deferredProxy(() -> selectedCommand());
    }

    /**
     * Returns the currently selected command.
     *
     * <p>If you plan on using this {@link Command} in a {@code Trigger} it is recommended to use
     * {@link #selectedCommandScheduler()} instead.
     *
     * @return The currently selected command.
     */
    public Command selectedCommand() {
        if (RobotBase.isSimulation() && nameAtGeneration == DO_NOTHING_NAME) {
            select(selected, true);
        }
        return generatedCommand;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        builder.publishConstBoolean(".controllable", true);
        builder.publishConstString("default", DO_NOTHING_NAME);
        builder.addStringArrayProperty("options", () -> options, null);
        builder.addStringProperty("selected", null, this::select);
        builder.addStringProperty("active", () -> select(selected), null);
    }

    @Override
    public void periodic() {
        Logger.processInputs(prefix + "/SmartDashboard", inputs);
    }
}
