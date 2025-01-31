package org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PitchConstants {
    public static double tickPerDegree = 5.8149;
    public static double kP = 0.017;
    public static double kD = 0.0006;


    public static double holdingkP = 0;
    public static double holdingkD = 0;
    public static double extensionVar = 0.5;
    public static double reverseSafePower = 0.15;
    ;



    public static double max_f = 0.145;

    public static double threshold =3;

    public static double lowThreshold = 10;
    public static double low_Value = -0.15;
    public static double max_Value = 0.45;

}
