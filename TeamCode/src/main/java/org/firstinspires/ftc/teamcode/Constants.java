package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

public class Constants {
    @Config
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        // --- Pollen & Nectar Constants ---
        public static final double POLLEN_TARGET_VELOCITY = 2250;  // flywheel RPM, keep below ~2000
        public static final double POLLEN_SERVO_DELAY_SEC = 1.5;     // longest wait for the wheel to reach speed
        public static final double POLLEN_SERVO_OPEN_SEC = 3.0;      // how long the servo is open and the intake feeds
        public static final double POLLEN_SERVO_INITIAL_POS = 0.29;   // closed (base) position
        public static final double POLLEN_SERVO_SECOND_POS = 0.35;    // open position (0.3 = 90 degrees of 300)

        public static final double NECTAR_TARGET_VELOCITY = 2000.0;  // flywheel RPM, keep below ~2000
        public static final double NECTAR_SERVO_DELAY_SEC = 0;
        public static final double NECTAR_SERVO_OPEN_SEC = 3.0;
        public static double NECTAR_SERVO_INITIAL_POS = 0.4;   // not used until Necter gets a servo
        public static final double NECTAR_SERVO_SECOND_POS = 0.55;
    }

    public static final class IntakeConstants {
        public static final double kIntakeOn = 1;

        public static final double kIntakeOn2 = 0.40;

        public static final double kIntakeOff = 0;
        public static final double kIntakeReverse = -0.7;
        public static final double kIntakeOnInAuto = 0.4;
        public static final double kIntakePlow = 0.75;   // intake speed the whole time the FlowPlow is down
    }

    @Config     // shows these on FTC Dashboard so they can be changed live
    public static final class VisionConstants {
        // The hive is a tag cluster. Names start with R/B for alliance (e.g. "RED SCORING"); the
        // cluster whose name contains this word (upper case) is the one we aim at.
        public static final String kHiveClusterKeyword = "SCORING";
        public static final String kHiveClusterKeyword2 = "AUDIENCE";

        // "public static" (not final) so Dashboard can change them
        public static double kAimKP = 0.02;           // turn power per degree of bearing error
        public static double kAimMaxTurn = 0.5;       // turn power cap
        public static double kAimMinTurn = 0.08;      // smallest turn power that still moves the robot
        public static double kAimToleranceDeg = 1.5;  // "lined up" if error is under this
        public static double kAimSettleMs = 100;      // must stay lined up this long before shooting
        public static double kAimTimeoutMs = 1500;    // give up aiming (and shoot anyway) after this
        public static double kAimOffsetDeg = 0;       // + if the shooter points to the right of the camera
        public static double kAimTurnSign = -1;       // flip to +1 if the robot turns away from the hive
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