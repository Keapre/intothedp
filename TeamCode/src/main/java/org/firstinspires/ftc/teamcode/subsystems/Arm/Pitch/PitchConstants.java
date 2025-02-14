package org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PitchConstants {
    public static double tickPerDegree = 5.8149;
    public static double kP = 0.0132;
    public static double kD = 0.00049;


    public static double holdingkP = 0;
    public static double holdingkD = 0;
    public static double extensionVar = 0.5;
    public static double reverseSafePower = -0.15;
    ;



    public static double max_f = 0.129;

    public static double threshold =3;


    public static double lowSpeedBasket = 0.2;
    public static double highSpeedBasket = 0.8;
    public static double highSpeedDown = 0.6;
    public static double lowSpeedDown = 0.1;

    public static double lowThreshold = 10;
    public static double low_Value = -0.15;
    public static double max_Value = 0.6;

}
