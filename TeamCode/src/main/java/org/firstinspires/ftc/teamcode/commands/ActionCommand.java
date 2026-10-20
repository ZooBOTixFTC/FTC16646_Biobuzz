package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

public class ActionCommand extends CommandBase {

    private final Action action;
    private boolean finished = false;

    /**
     * Wraps a Roadrunner Action into an FTCLib Command.
     *
     * @@param action The Roadrunner 1.0 Action to execute (e.g., trajectory)
     * @param requirements Subsystems required by this command (typically your drivetrain)
     */
    public ActionCommand(Action action, Subsystem... requirements) {
        this.action = action;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();

        // Action.run() returns true while still running, and false when finished
        finished = !action.run(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}