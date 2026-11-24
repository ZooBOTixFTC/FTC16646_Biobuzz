package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SUB_AimTuner;

/** "Teleop Blue" plus the gamepad 2 aim tuner. Everything else is exactly the same. */
@TeleOp(name = "Teleop Blue + Aim Tuner", group = "Teleop Blue")
public class TELEOP_FieldCentricBlueTuner extends TELEOP_FieldCentricBlue {
    @Override
    public void configureButtonBindings() {
        super.configureButtonBindings();
        new SUB_AimTuner(m_robot, telemetry, gamepad1, gamepad2);
    }
}
