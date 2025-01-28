package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;

@Config
public class ArmConstants {
    //take from each file from the folder armstates and them as variables just the pitch and extension
    public static double DEFAULT_EXTENSION = 0;
    public static double DEFAULT_PITCH = 0;

    public static double HIGHBASKET_EXTENSION = 530;
    public static double HIGHBASKET_PITCH = 555;

    public static double INTAKING_EXTENSION = 0;
    public static double INTAKING_PITCH = 0;
    public static double START_EXTENSION = 0;
    public static double START_PITCH = 170;

    public static double SPECIMEN_EXTENSION = 430;
    public static double SPECIMEN_PITCH = 320;

    public static double SPECIMENGARD_EXTENSION = 0;
    public static double SPECIMENGARD_PITCH = 86;

    public static double SPECIMENSLAM_EXTENSION = 0;
    public static double SPECIMENSLAM_PITCH = 670;

    public static Claw.tiltMode MID_TILT = Claw.tiltMode.MID;
    public static Claw.tiltMode UP2_TILT = Claw.tiltMode.UP2;
    public static Claw.tiltMode GARD_TILT = Claw.tiltMode.MIDGARD;
    public static Claw.tiltMode DOWN_TILT = Claw.tiltMode.DOWN;
    public static Claw.tiltMode UP_TILT = Claw.tiltMode.UP;

    public static Claw.CLAWPOS CLOSE_CLAW = Claw.CLAWPOS.CLOSE;
    public static Claw.CLAWPOS OPEN_CLAW = Claw.CLAWPOS.OPEN;
    public static Claw.RotateMode ORIZONTAL_ROTATE = Claw.RotateMode.ORIZONTAL;
    public static Claw.RotateMode VERTICAL_ROTATE = Claw.RotateMode.VERTICAL;

}
