package org.firstinspires.ftc.teamcode.roadrunner;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Saves and loads the tuned drivetrain values in MecanumDrive.PARAMS
 * to a file on the Control Hub (/sdcard/FIRST/settings/drive_params.txt).
 *
 * MecanumDrive loads this file automatically when the app starts,
 * so every OpMode (teleop, autos, tuners) uses the same saved values.
 */
public final class DriveParamsStore {

    private static final String SAVE_FILE_NAME = "drive_params.txt";

    /** Shown on telemetry so you know what happened. */
    public static String status = "Using code defaults";

    private DriveParamsStore() {}

    /** Saves the current MecanumDrive.PARAMS values. */
    public static void save() {
        MecanumDrive.Params p = MecanumDrive.PARAMS;
        String data =
                "inPerTick=" + p.inPerTick + "\n" +
                        "lateralInPerTick=" + p.lateralInPerTick + "\n" +
                        "trackWidthTicks=" + p.trackWidthTicks + "\n" +
                        "kS=" + p.kS + "\n" +
                        "kV=" + p.kV + "\n" +
                        "kA=" + p.kA + "\n" +
                        "axialGain=" + p.axialGain + "\n" +
                        "lateralGain=" + p.lateralGain + "\n" +
                        "headingGain=" + p.headingGain + "\n" +
                        "axialVelGain=" + p.axialVelGain + "\n" +
                        "lateralVelGain=" + p.lateralVelGain + "\n" +
                        "headingVelGain=" + p.headingVelGain + "\n";
        try {
            ReadWriteFile.writeFile(getFile(), data);
            status = "SAVED  ticks/in=" + String.format("%.2f", 1.0 / p.inPerTick);
        } catch (Exception e) {
            status = "SAVE FAILED: " + e.getMessage();
        }
    }

    /** Loads saved values into MecanumDrive.PARAMS if the file exists. */
    public static boolean load() {
        try {
            File file = getFile();
            if (!file.exists()) {
                status = "Using code defaults";
                return false;
            }

            MecanumDrive.Params p = MecanumDrive.PARAMS;
            for (String line : ReadWriteFile.readFile(file).split("\n")) {
                String[] kv = line.trim().split("=");
                if (kv.length != 2) continue;
                double value = Double.parseDouble(kv[1].trim());

                switch (kv[0].trim()) {
                    case "inPerTick":        p.inPerTick = value; break;
                    case "lateralInPerTick": p.lateralInPerTick = value; break;
                    case "trackWidthTicks":  p.trackWidthTicks = value; break;
                    case "kS":               p.kS = value; break;
                    case "kV":               p.kV = value; break;
                    case "kA":               p.kA = value; break;
                    case "axialGain":        p.axialGain = value; break;
                    case "lateralGain":      p.lateralGain = value; break;
                    case "headingGain":      p.headingGain = value; break;
                    case "axialVelGain":     p.axialVelGain = value; break;
                    case "lateralVelGain":   p.lateralVelGain = value; break;
                    case "headingVelGain":   p.headingVelGain = value; break;
                }
            }
            status = "Loaded saved values";
            return true;
        } catch (Exception e) {
            status = "LOAD FAILED (using code defaults): " + e.getMessage();
            return false;
        }
    }

    /** Deletes the saved file so the code defaults are used after the next app restart. */
    public static void clear() {
        File file = getFile();
        status = (file.exists() && file.delete())
                ? "Saved file deleted (defaults after restart)"
                : "No saved file to delete";
    }

    private static File getFile() {
        return AppUtil.getInstance().getSettingsFile(SAVE_FILE_NAME);
    }
}