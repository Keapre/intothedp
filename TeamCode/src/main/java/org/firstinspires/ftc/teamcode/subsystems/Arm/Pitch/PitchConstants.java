package org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PitchConstants {
    public static double tickPerDegree = 6.10352;
    public static double kP = 0.013;
    public static double kD = 0.00025;

    public static double max_f = 0.2;

    public static double threshold = 5;

    public static double lowThreshold = 10;
    public static double low_Value = -0.39;
    public static double max_Value = -0.39;

}
