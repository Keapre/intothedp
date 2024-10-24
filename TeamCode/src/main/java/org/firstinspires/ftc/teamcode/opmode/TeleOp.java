package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

public class TeleOp extends OpMode {
    Robot robot;
    GamePadController gamepadd;

    @Override
    public void init() {
        robot = new Robot(this,false);
        gamepadd = new GamePadController(gamepad1);
    }

    @Override
    public void loop() {
        robot.mecanum.setMotorPowersFromGamepad(gamepadd,1,false,true);
    }
}
