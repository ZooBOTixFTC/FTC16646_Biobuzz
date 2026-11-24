package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SUB_AimTuner;

/** "Teleop Red" plus the gamepad 2 aim tuner. Everything else is exactly the same. */
@TeleOp(name = "Teleop Red + Aim Tuner", group = "Teleop Red")
public class TELEOP_FieldCentricTuner extends TELEOP_FieldCentric {
    @Override
    public void configureButtonBindings() {
        super.configureButtonBindings();
        new SUB_AimTuner(m_robot, telemetry, gamepad1, gamepad2);   // registers itself, runs every loop
    }
}
