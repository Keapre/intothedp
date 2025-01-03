package org.firstinspires.ftc.teamcode.subsystems.Arm.Extension;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ExtensionConstants {
    //PID
    public static double kP = 0.04;
    public static double kI = 0.0;
    public static double kD = 0.0007;

    public static double kf = 0.14; //probabil o sa actioneze ca un ks
    //thresholds

    public static double pointThreeshold = 2;

    public static double dynamicTimerThreshold = 3;
}
