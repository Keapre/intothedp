package org.firstinspires.ftc.teamcode.subsystems.Arm.Extension;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ExtensionConstants {
    //PID
    public static double kP = 0.035;
    public static double kI = 0.0;
    public static double kD = 0.0007;
    public static double idlePower = 0;
    public static double maxPower = 0.75;
    public static double minPower = -1;
    public static double valueSHit = 0;//should lower this fr fr;

    public static double kf = 0.055; //0.20 //probabil o sa actioneze ca un ks
    //thresholds

    public static double pointThreeshold = 9;
    public static double at0Threeshold = 6;
    public static double at0positionPitch = 40;

    public static double dynamicTimerThreshold = 3;
}
