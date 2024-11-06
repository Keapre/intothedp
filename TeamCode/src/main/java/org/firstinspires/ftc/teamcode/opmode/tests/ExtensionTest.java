package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@TeleOp(name = "extension")
@Config
public class ExtensionTest extends LinearOpMode {

    public static double power = 0.8;

    public static double sign = -1;
    GamePadController g1;
    DcMotorEx motor1;
    public enum Mode {
        extended,
        idle,
        retracting
    }
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class,"extend");
        g1 = new GamePadController(gamepad1);
        Mode mode = Mode.idle;
        waitForStart();

        while(opModeIsActive()) {
            g1.update();
            if(g1.leftBumper()) {
                mode = Mode.retracting;
            }else if(g1.rightBumper()) {
                mode = Mode.extended;
            }else {
                mode = Mode.idle;
            }
            switch (mode) {
                case idle:
                    motor1.setPower(0);
                    break;
                case extended:
                    motor1.setPower(sign * power);
                    break;
                case retracting:
                    motor1.setPower(sign * -1 * power);
                    break;
            }
            telemetry.addData("mode",mode);
            telemetry.addData("power",power);
            telemetry.update();
        }
    }
}
