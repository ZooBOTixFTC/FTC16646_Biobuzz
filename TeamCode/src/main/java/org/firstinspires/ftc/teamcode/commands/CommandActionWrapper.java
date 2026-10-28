package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;

public class CommandActionWrapper implements Action {
    private final Command command;
    private boolean initialized = false;

    public CommandActionWrapper(Command command) {
        this.command = command;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            command.initialize();
            initialized = true;
        }

        command.execute();

        if (command.isFinished()) {
            command.end(false);
            return false; // Tells Road Runner this step is done
        }
        return true; // Keeps looping this command until it finishes
    }
}