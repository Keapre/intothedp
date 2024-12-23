package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Utils.Caching.OPColorSensor;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@TeleOp(name = "Miscoci claw")
public class CRservoTEst extends LinearOpMode {
    public static double power = 1;
    com.qualcomm.robotcore.hardware.CRServo crServo;


    OPColorSensor rev;
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rev = new OPColorSensor(this.hardwareMap,"intakeSensor");
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
            telemetry.addData("green",rev.green());
            telemetry.addData("blue",rev.blue());
            telemetry.addData("red",rev.red());
            telemetry.addData("is Blue",rev.isBlue());
            telemetry.addData("is Red",rev.isRed());
            telemetry.addData("is Yellow",rev.isYellow());
            telemetry.addData("distance",rev.distance());
            telemetry.addData("took it",rev.tookit());
            telemetry.addData("argb",rev.normalizedValues() );
            telemetry.addData("rgb",rev.rgb());
            telemetry.addData("Color",rev.getColor());
            telemetry.update();
        }
    }
}
