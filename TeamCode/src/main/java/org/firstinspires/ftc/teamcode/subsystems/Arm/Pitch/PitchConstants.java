package org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PitchConstants {
    public static double tickPerDegree = 5.58959;
    public static double kP = 0.015;
    public static double kD = 0.0004;


    public static double holdingkP = 0;
    public static double holdingkD = 0;
    public static double extensionVar = 0.5;
    ;



    public static double max_f = 0.16;

    public static double threshold =3;

    public static double lowThreshold = 10;
    public static double low_Value = -0.30;
    public static double max_Value = 0.5;

}
