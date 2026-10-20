package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Red Robot Centric", group = "Teleop Red")
public class TELEOP_RobotCentric extends TELEOP_FieldCentric {
    @Override
    public void setSide(){
        m_robot.setRedSide();
        m_robot.drivetrain.setFieldCentric(false);
    }
}
