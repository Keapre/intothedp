package org.firstinspires.ftc.teamcode.subsystems.Hang;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingServo;

public class Hang {
    CachingServo hangServoLeft, hangServoRight;


    public enum HangPosition {
        UP,
        DOWN
    }

    public static double hang_left_up = 0.0;
    public static double hang_left_down = 0.0;
    public static double hang_right_up = 0.0;
    public static double hang_right_down = 0.0; // TODO: should be sync

    public HangPosition hangPosition = HangPosition.DOWN;
    public Hang(HardwareMap hardwareMap) {
        hangServoLeft = new CachingServo(hardwareMap.servo.get("hangServoLeft"));
        hangServoRight = new CachingServo(hardwareMap.servo.get("hangServoRight"));

        hangServoLeft.setPosition(hang_left_down);
        hangServoRight.setPosition(hang_right_down);
    }

    public void update() {
        switch (hangPosition) {
            case UP:
                hangServoLeft.setPosition(hang_left_up);
                hangServoRight.setPosition(hang_right_up);
                break;
            case DOWN:
                hangServoLeft.setPosition(hang_left_down);
                hangServoRight.setPosition(hang_right_down);
                break;
            default:
                break;
        }
    }
}
