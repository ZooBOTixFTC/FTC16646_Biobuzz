package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Blue", group = "Teleop Blue")
public class TELEOP_FieldCentricBlue extends TELEOP_FieldCentric {
    @Override
    public void setSide(){
        m_robot.setBlueSide();
    }
}
