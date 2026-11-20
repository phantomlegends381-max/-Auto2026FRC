package frc.robot.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Non-blocking lifecycle wrapper for the selected PathPlanner autonomous.
 * The drivetrain is stopped if the auto finishes, times out, or is interrupted.
 */
public final class AutonomousFSMCommand extends Command {
    public enum State {
        INIT,
        RUN_AUTO,
        DONE,
        ABORT
    }

    private static final double AUTO_TIMEOUT_SECONDS = 19.0;

    private final Supplier<Command> autoSupplier;
    private final Runnable safeStop;
    private final Timer stateTimer = new Timer();

    private State state = State.INIT;
    private Command activeAuto;

    public AutonomousFSMCommand(Supplier<Command> autoSupplier, Runnable safeStop, Subsystem... requirements) {
        this.autoSupplier = Objects.requireNonNull(autoSupplier);
        this.safeStop = Objects.requireNonNull(safeStop);
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        activeAuto = null;
        transitionTo(State.INIT);

        activeAuto = autoSupplier.get();
        if (activeAuto == null) {
            transitionTo(State.ABORT);
            safeStop.run();
            return;
        }

        transitionTo(State.RUN_AUTO);
        activeAuto.initialize();
    }

    @Override
    public void execute() {
        if (state != State.RUN_AUTO || activeAuto == null) return;

        activeAuto.execute();
        if (activeAuto.isFinished()) {
            activeAuto.end(false);
            activeAuto = null;
            transitionTo(State.DONE);
            safeStop.run();
        } else if (stateTimer.hasElapsed(AUTO_TIMEOUT_SECONDS)) {
            activeAuto.end(true);
            activeAuto = null;
            transitionTo(State.ABORT);
            safeStop.run();
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE || state == State.ABORT;
    }

    @Override
    public void end(boolean interrupted) {
        if (activeAuto != null) {
            activeAuto.end(true);
            activeAuto = null;
        }
        safeStop.run();
        if (interrupted) transitionTo(State.ABORT);
    }

    public State getState() {
        return state;
    }

    private void transitionTo(State next) {
        state = next;
        stateTimer.restart();
        SmartDashboard.putString("Auto FSM State", state.name());
    }
}
