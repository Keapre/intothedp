package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.MecanumUtil;
import org.firstinspires.ftc.teamcode.Utils.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

public class MecanumTest implements Subsystem {
    public static double rotateNormal = 0.65;

    public static double slowMode = 0.6;
    public boolean slow_mode = false;

    public double[] motorPowers = new double[]{
            0,
            0,
            0,
            0,
    };
    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public MecanumTest(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront =hardwareMap.get(DcMotorEx.class, "rightFront");

        //TODO:reverse motor dumbass
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorPowersFromGamepad(GamePadController gg, double scale, boolean reverseFront, boolean customCurve) {
        MecanumUtil.Motion motion;

        if(gg.leftStickButtonOnce()){
            slow_mode = !slow_mode;
        }
        double left_stick_x = gg.left_stick_x;
        double left_stick_y = gg.left_stick_y;
        double right_stick_x = rotateNormal * gg.right_stick_x;
        double right_stick_y = rotateNormal * gg.right_stick_y;

        if(slow_mode) {
            left_stick_x*=slowMode;
            right_stick_y*=slowMode;
            right_stick_x*=slowMode;
            right_stick_y*=slowMode;
        }
        motion = MecanumUtil.joystickToMotion(left_stick_x, left_stick_y,
                right_stick_x, right_stick_y, reverseFront, customCurve);

        MecanumUtil.Wheels wh = MecanumUtil.motionToWheels(motion).scaleWheelPower(scale); // Use full forward speed on 19:1 motors
/*        motorPowers[0] = ffMotor.compute(wh.frontLeft,PARAMS.maxProfileAccel);
        motorPowers[1] = ffMotor.compute(wh.backLeft,PARAMS.maxProfileAccel);
        motorPowers[2] = ffMotor.compute(wh.backRight,PARAMS.maxProfileAccel);
        motorPowers[3] = ffMotor.compute(wh.frontRight,PARAMS.maxProfileAccel);*/
        motorPowers[0] = wh.frontLeft;
        motorPowers[1] = wh.backLeft;
        motorPowers[2] = wh.backRight;
        motorPowers[3] = wh.frontRight;
        leftFront.setPower(wh.frontLeft);
        rightFront.setPower(wh.frontRight);
        leftBack.setPower(wh.backLeft);
        rightBack.setPower(wh.backRight);
    }
    public void setMotorPowers(double v, double v1, double v2, double v3) {
/*        leftFront.setPower(motorPowers[0] = v);
        leftBack.setPower(motorPowers[1] = v1);
        rightBack.setPower(motorPowers[2] = v2);
        rightFront.setPower(motorPowers[3] = v3);*/
//
        leftFront.setPower(motorPowers[0] = v);
        leftBack.setPower(motorPowers[1] = v1);
        rightBack.setPower(motorPowers[2] = v2);
        rightFront.setPower(motorPowers[3] = v3);
    }

    @Override
    public void update() {

    }
}
