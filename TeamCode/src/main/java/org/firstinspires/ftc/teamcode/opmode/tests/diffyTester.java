package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@Disabled

@Config
@TeleOp(name = "diffy")
public class diffyTester extends LinearOpMode {

    Servo diffyLeft,diffyRight;

    public static double targetLeft = 0.5;
    public static double targetRight = 0.5;

    public static double actual_Left = 0.5;
    public static double actual_Right = 0.5;
    GamePadController gg;
    @Override
    public void runOpMode() throws InterruptedException {
        diffyLeft = hardwareMap.get(Servo.class,"diffyLeft");
        diffyRight = hardwareMap.get(Servo.class,"diffyRight");

        gg = new GamePadController(gamepad1);
        waitForStart();

        while (opModeIsActive()) {
            gg.update();
            if(gg.xOnce()) {
                diffyLeft.setPosition(targetLeft);
                diffyRight.setPosition(targetRight);
            }
        }

    }
}
