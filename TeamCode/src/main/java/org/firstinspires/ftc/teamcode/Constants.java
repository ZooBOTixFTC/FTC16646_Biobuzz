package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        public static final double kFarVel = -0.5;
        public static final double kFarthestVel = -0.40 ;
        public static final double kCloseVel = -.40;
        public static final double kMidFieldVel = -0.5;
        public static final double kPreRev = -.25;

        public static final double kTolerance = 0.01;
        public static final double kMaxVel = 2100;
        public static final double kP = 10;
        public static final double kF = 0.88;
        // --- Pollen & Nectar Constants ---
        public static final double POLLEN_MIN_VELOCITY = 2100;     // speed between shots (0 = stops)
        public static final double POLLEN_TARGET_VELOCITY = 1725;  // flywheel RPM, keep below ~2000
        public static final double POLLEN_SERVO_DELAY_SEC = 0;     // longest wait for the wheel to reach speed
        public static final double POLLEN_SERVO_OPEN_SEC = 3.0;      // how long the servo is open and the intake feeds
        public static final double POLLEN_SERVO_INITIAL_POS = 0.25;   // closed (base) position
        public static final double POLLEN_SERVO_SECOND_POS = 0.35;    // open position (0.3 = 90 degrees of 300)

        public static final double NECTAR_MIN_VELOCITY = 2125.0;     // speed between shots (0 = stops)
        public static final double NECTAR_TARGET_VELOCITY = 2125.0;  // flywheel RPM, keep below ~2000
        public static final double NECTAR_SERVO_DELAY_SEC = 0;
        public static final double NECTAR_SERVO_OPEN_SEC = 3.0;
        public static final double NECTAR_SERVO_INITIAL_POS = 0.35;   // not used until Necter gets a servo
        public static final double NECTAR_SERVO_SECOND_POS = 0.6;
    }

    public static final class IntakeConstants {
        public static final double kIntakeOn = 0.5;
        public static final double kIntakeAuto = 0.9;
        public static final double kIntakeOff = 0;
        public static final double kIntakeReverse = -0.67;
    }

    public static final class TransferConstants {
        public static final double kTransferOn = 0.75;
        public static final double kTransferAuto = 0.9;
        public static final double kTransferOff = 0;
        public static final double kTransferReverse = -0.67;
    }

    public static class SystemConstants{
        public static final int kSysUpdateRateHz = 10;
    }
}






