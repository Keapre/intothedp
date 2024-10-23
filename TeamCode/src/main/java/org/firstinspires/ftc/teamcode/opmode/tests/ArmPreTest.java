package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Caching.OPColorSensor;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@Config
@TeleOp(name = "MiscociOuttake")
public class ArmPreTest extends LinearOpMode {

    Servo claw,rotate;
    OPColorSensor clSensor;

    GamePadController gamepadd;

    public static double clawClosed = 0.75;
    public static double clawOpened = 0.4;
    long lastChecked = System.currentTimeMillis();
    public static double threesholdTransition = 550;
    public static double clawThreeshold = 15;
    public static  boolean usedSensor = true;

    public static double ratioWidth = 1.0;

    public static double verticalPose = 0.5;
    public static double orizontalPose = 0.5;

    enum RotateMode {
        VERTICAL(verticalPose),
        ORIZONTAL(orizontalPose);

        double x;

         RotateMode(double x) {
            this.x = x;
        }
    }



    enum ServoMode{
        CLOSED,
        PRE_CLOSED,
        OPEN
    }

    @Override
    public void runOpMode() throws InterruptedException {
        ServoMode modeServo = ServoMode.OPEN;

        claw = hardwareMap.get(Servo.class,"Claw");
        rotate = hardwareMap.get(Servo.class,"rotate");
        RotateMode rMode = RotateMode.ORIZONTAL;
        clSensor =new OPColorSensor(hardwareMap.get(ColorRangeSensor.class,"sensor"));
        gamepadd = new GamePadController(gamepad1);
        waitForStart();

        while(opModeIsActive()) {
            clSensor.update();
            gamepadd.update();
            if(gamepadd.aOnce()) {
                if(modeServo == ServoMode.CLOSED){
                    modeServo = ServoMode.OPEN;
                    lastChecked = System.currentTimeMillis();
                }
                else modeServo = ServoMode.CLOSED;
            }
            if(gamepadd.bOnce()) {
                usedSensor = !usedSensor;
            }

            switch (modeServo){
                case CLOSED:
                    claw.setPosition(clawClosed);
                    break;
                case OPEN:
                    if(usedSensor && clSensor.getDistance() < clawThreeshold && System.currentTimeMillis() - lastChecked > threesholdTransition) {
                        lastChecked = System.currentTimeMillis();
                        modeServo = ServoMode.CLOSED;
                        break;
                    }
                    claw.setPosition(clawOpened);
                    break;
                case PRE_CLOSED:
                    if(System.currentTimeMillis() - lastChecked >= threesholdTransition) {
                        modeServo = ServoMode.CLOSED;
                    }
                    break;
                default:
                    break;
            }
            telemetry.addData("distance",clSensor.getDistance());
            telemetry.addData("mode",modeServo);
            telemetry.addData("use Sensor:",usedSensor);
            telemetry.update();
        }

    }
}
