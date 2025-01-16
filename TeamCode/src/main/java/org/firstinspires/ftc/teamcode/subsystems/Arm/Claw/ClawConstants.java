package org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConstants {
    //Claw positions
    public static double clawOpen = 0.22;
    public static double clawClose = 0.58;

    //Diffy positions

    public static class DiffyPositions {
        public double left;
        public double right;

        public DiffyPositions(double l, double r) {
            left = l;
            right = r;
        }
    }

    public static DiffyPositions down_horizontal = new DiffyPositions(0.18, 0.97);
    public static DiffyPositions down_vertical = new DiffyPositions(0, 0.79);
    public static DiffyPositions mid = new DiffyPositions(0.73, 0.39);
    public static DiffyPositions up = new DiffyPositions(1, 0.11);
    public static DiffyPositions up2 = new DiffyPositions(0.86, 0.25);
    public static DiffyPositions slamPos = new DiffyPositions(0.32, 0.02);



}