package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@TeleOp(name = "Miscoci claw")
public class CRservoTEst extends LinearOpMode {
    public static double power = 1;
    com.qualcomm.robotcore.hardware.CRServo crServo;
    GamePadController gg;
    enum Mode{
        REVERSE,
        FORWARD,
        IDLE
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Mode mode = Mode.IDLE;
        crServo = hardwareMap.get(CRServo.class,"test");
        gg = new GamePadController(gamepad1);

        waitForStart();
        while (opModeIsActive()) {
            gg.update();

            if(gg.a()) {
                mode = Mode.FORWARD;
            }else if(gg.b()) {
                mode = Mode.REVERSE;
            }else {
                mode = Mode.IDLE;
            }

            switch (mode) {
                case IDLE:
                    crServo.setPower(0);
                    break;
                case FORWARD:
                    crServo.setPower(power);
                    break;
                case REVERSE:
                    crServo.setPower(-power);
                    break;
            }
        }
    }
}
