package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class tuneFF extends LinearOpMode {

    public static double tickPerDegree = 9.366695;
    public static double kGpowerInceput = 0.037;

    DcMotorEx motor,motorE;
    public static double signFeed = -1;
    public static double ff = 0;
    public static double angle(double ticks) {
        return (-ticks) / tickPerDegree;
    }

    public static double kv = 0,ka = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motorE = hardwareMap.get(DcMotorEx.class, "pivot1");
        motor = hardwareMap.get(DcMotorEx.class, "pivot2");

        double offset = motorE.getCurrentPosition();
        waitForStart();
        while(opModeIsActive()) {
            double Angle = angle(motorE.getCurrentPosition() - offset);
            ff = signFeed * kGpowerInceput * Math.cos(Math.toRadians(Angle));

        }
    }
}
