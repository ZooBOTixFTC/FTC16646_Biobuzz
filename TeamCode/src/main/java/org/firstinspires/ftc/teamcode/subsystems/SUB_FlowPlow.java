package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * FlowPlow: two servos that move together. They face each other, so one is inverted.
 *   FlowPlowToTheCloud()  = up
 *   FlowPlowToTheGround() = down
 */
public class SUB_FlowPlow extends SubsystemBase {

    // ---- Tune these ----
    public static final double CLOUD_POS = 0.1;    // up position
    public static final double GROUND_POS = 0.61;   // down position

    private final Servo leftServo;
    private final Servo rightServo;

    public SUB_FlowPlow(OpMode opMode) {
        leftServo = opMode.hardwareMap.get(Servo.class, "flowerPlowLeftServo");
        rightServo = opMode.hardwareMap.get(Servo.class, "flowerPlowRightServo");

        // The two servos are mirrored, so one of them is inverted
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        FlowPlowToTheCloud();   // start down
    }

    /** Moves FlowPlow up. */
    public void FlowPlowToTheCloud() {
        setBoth(CLOUD_POS);
    }

    /** Moves FlowPlow down. */
    public void FlowPlowToTheGround() {
        setBoth(GROUND_POS);
    }

    private void setBoth(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("FlowPlow L/R", "%.2f / %.2f",
                leftServo.getPosition(), rightServo.getPosition());
    }
}