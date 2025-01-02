package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public static boolean IS_AUTO = true;
    public static boolean autoBasket = false;

    public static boolean IS_RED = false;

    public static boolean isAuto() {
        return IS_AUTO;
    }

    public static boolean isAutoBasket() {
        return autoBasket;
    }
}
