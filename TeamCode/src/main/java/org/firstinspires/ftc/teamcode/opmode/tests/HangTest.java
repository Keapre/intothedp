package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@Config
@TeleOp
public class HangTest extends LinearOpMode {
    DcMotorEx hang;
    GamePadController gg;
    public static double sign = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        hang = hardwareMap.get(DcMotorEx.class,"hang");
        gg = new GamePadController(gamepad1);

        waitForStart();
        while(opModeIsActive()) {
            gg.update();
            hang.setPower(-1 * (gg.right_trigger + -gg.left_trigger));
        }
    }
}
