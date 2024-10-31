package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@TeleOp(name = "halloween party::)))))")
public class PitchTest extends LinearOpMode {
    DigitalChannel limitSwitch;
    DcMotorEx motorE, motor;

    GamePadController gg;

    @Override
    public void runOpMode() throws InterruptedException {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");
        motorE = hardwareMap.get(DcMotorEx.class, "pivot1");
        motor = hardwareMap.get(DcMotorEx.class, "pivot2");
        gg = new GamePadController(gamepad1);
       // motorE.setDirection(DcMotorSimple.Direction.REVERSE);
        motorE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()) {
            gg.update();

            double power = -gg.left_stick_y;

            if (limitSwitch.getState() == false) {
                telemetry.addData("Button", "PRESSED");
            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }
            motorE.setPower(power);
            motor.setPower(power);
            telemetry.addData("power",power);
            telemetry.addData("position", motorE.getCurrentPosition());
            telemetry.update();
        }
    }
}
