package org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ClawConstants {
    //Claw positions
    public static double clawOpen = 0.08;
    public static double clawClose = 0.68;

    //Diffy positions

    public static class DiffyPositions {
        public double left;
        public double right;

        public DiffyPositions(double l, double r) {
            left = l;
            right = r;
        }
    }

    public static DiffyPositions down_horizontal = new DiffyPositions(0.15, 0.85);
    public static DiffyPositions down_vertical = new DiffyPositions(0.26, 0.97);
    public static DiffyPositions mid = new DiffyPositions(0.5, 0.5);
    public static DiffyPositions up = new DiffyPositions(0.93, 0.12);
    public static DiffyPositions up_vert = new DiffyPositions(0.75, 0.47);
    public static DiffyPositions midGard = new DiffyPositions(0.43,0.57);
    public static DiffyPositions up2 = new DiffyPositions(0.8, 0.25 );
   // public static DiffyPositions slamPos = new DiffyPositions(0.32, 0.02);



}