package org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConstants {
    //Claw positions
    public static double clawOpen = 0.17;
    public static double clawClose = 0.72;

    //Diffy positions

    public static class DiffyPositions {
        public double left;
        public double right;

        public DiffyPositions(double l, double r) {
            left = l;
            right = r;
        }
    }

    public static DiffyPositions down_horizontal = new DiffyPositions(0.22, 0.83);
    public static DiffyPositions down_vertical = new DiffyPositions(0.06, 0.69);
    public static DiffyPositions mid = new DiffyPositions(0.65, 0.4);
    public static DiffyPositions up = new DiffyPositions(0.9, 0.15);
    public static DiffyPositions midGard = new DiffyPositions(0.64,0.41); // 0.70 cu 0.39
    public static DiffyPositions up2 = new DiffyPositions(0.77, 0.28);
   // public static DiffyPositions slamPos = new DiffyPositions(0.32, 0.02);



}