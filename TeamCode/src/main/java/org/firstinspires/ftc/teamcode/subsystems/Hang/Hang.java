package org.firstinspires.ftc.teamcode.subsystems.Hang;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Utils.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@Config
public class Hang implements Subsystem {
    CachingServo hangServoLeft, hangServoRight;
    DcMotorEx motor;

    public enum State {
        IDLE,
        CLIMBING,
        NO_START
    }

    double power = 0;
    public void changePower(GamePadController gg) {
        power = gg.left_trigger - gg.right_trigger;
    }

    public static double idlePower = 0.2;
    public boolean idleActivated = false;

    @Override
    public void update() {
        if(idleActivated) {
            power = idlePower;
        }
        motor.setPower(power);
    }


    public enum HangPosition {
        UP,
        DOWN
    }

    public static double hang_left_up = 0.3;
    public static double hang_left_down = 0.65;
    public static double hang_right_up = 0.6;

   public State state = State.NO_START;
    public static double hang_right_down = 0.35; // TODO: should be sync

    public HangPosition hangPosition = HangPosition.DOWN;
    public Hang(HardwareMap hardwareMap) {
        hangServoLeft = new CachingServo(hardwareMap.servo.get("hangleft"));
        hangServoRight = new CachingServo(hardwareMap.servo.get("hangright"));


        motor = hardwareMap.get(DcMotorEx.class, "hang");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangServoRight.setPosition(hang_right_down);
        hangServoLeft.setPosition(hang_left_down);

    }


    public void startClimb() {
        idleActivated = !idleActivated;
    }

    public void setServo() {
        hangServoLeft.setPosition(hang_left_up);
        hangServoRight.setPosition(hang_right_up);
    }

    public void setCloseServo() {
        hangServoLeft.setPosition(hang_left_down);
        hangServoRight.setPosition(hang_right_down);
    }
}
