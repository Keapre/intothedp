package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "mergi in fata")
public class AutoTest extends LinearOpMode {

    public static double power = 1;
    public static double timerSec = 4;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while (opModeIsActive()) {
            while(timer.time() > timerSec) {
                leftFront.setPower(power);
                leftBack.setPower(power);
                rightFront.setPower(power);
                rightBack.setPower(power);
            }
        }
    }
}
