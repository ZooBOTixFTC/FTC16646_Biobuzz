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
        public static double shootingPollenRPM = 1800;
        public static double shootingNectarRPM = 1800;
    }

    public static final class IntakeConstants {
        public static final double kIntakeOn = 0.8;
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


