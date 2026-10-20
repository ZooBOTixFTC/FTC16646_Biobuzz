package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teleop Blue Robot Centric", group = "Teleop Blue")
public class TELEOP_RobotCentricBlue extends TELEOP_RobotCentric {
    @Override
    public void setSide(){
        m_robot.setBlueSide();
        m_robot.drivetrain.setFieldCentric(false);
    }
}
