package org.firstinspires.ftc.teamcode.opmode.tuning;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Caching.OPColorSensor;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Drive;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;

import java.util.List;

@Config
@TeleOp(name = "Auto aim")
public class AutoAim extends OpMode {

    OPColorSensor revColor;
    public static double overShootVariable = 160;
    public static double kP = 0.0095, kD = 0.8;
    public static double sign = 1;
    public static double min_command = 0.095;
    public static double threeshold = 1;
    public static double power = 0.3;
    Limelight3A lm;
    Robot robot;
    public static double midTime = 200;
    Drive dt;
    States currentState = null;
    public static double powerExtension = 0.29;
    public static double safeSpeed = 0.2;
    GamePadController gg;
    boolean pid = false;
    public static double timerSecond = 40;
    boolean vertical = false;
    enum States {
        CHANGING_ANGLE,
        WAIT_FOR_CLAW,
        EXTENDING,
        STOP,
        IDLE
    }
    double degrees = 0;
    public static double checkLimit = 7;
    double tx,ty;
    double target = 0;
    boolean entered = false;
    int counterV = 0,counterH = 0;
    double desired_ty = 1e9;
    ElapsedTime timerMidClaw = null;
    public double rectangleOrientation(LLResult result) {
        List<List<Double>> targetCorners = result.getColorResults().get(0).getTargetCorners();


        double minX=1e9,maxX=-1e9,minY=1e9,maxY=-1e9;
//        xA = targetCorners.get(0).get(0);
//        yA = targetCorners.get(0).get(1);
        int size = targetCorners.size();
        telemetry.addData("corners",targetCorners.toString());
//        xB = targetCorners.get(1).get(0);
//        yB = targetCorners.get(1).get(1);
//
//        xC = targetCorners.get(2).get(0);
//        yC = targetCorners.get(2).get(1);
//
//        xD = targetCorners.get(2).get(0);
//        yD = targetCorners.get(2).get(1);

        for(int i = 0; i < size; i++) {
            double x = targetCorners.get(i).get(0);
            double y = targetCorners.get(i).get(1);
            minX = Math.min(minX,x);
            maxX = Math.max(maxX,x);
            minY = Math.min(minY,y);
            maxY = Math.max(maxY,y);
        }

//
//        double x = xB - xA;
//        double y = yC - yA;
//        double angle = Math.atan2(y,x);


//
        double length = maxX - minX;
        double witdth = maxY - minY;
        telemetry.addData("width",witdth);
        telemetry.addData("length",length);
        if(length > witdth) {
            counterV++;
            telemetry.addData("Orientation","Horizontal");
        }else {
            counterH++;
            telemetry.addData("Orientation","Vertical");
        }
        return 0;
    }
    @Override
    public void init() {
        lm = hardwareMap.get(Limelight3A.class, "limelight");
        revColor = new OPColorSensor(hardwareMap, "color");
        robot = new Robot(this, new Pose2d(0, 0, 0));
        telemetry.setMsTransmissionInterval(11);
        currentState = States.IDLE;
        lm.pipelineSwitch(0);
        gg = new GamePadController(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dt = new Drive(hardwareMap, new Pose2d(0, 0, 0), false);
    }

    @Override
    public void start() {
        robot.start();
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        ElapsedTime timerClaw = null;
        lm.start();
    }
    @Override
    public void loop() {
        gg.update();
        if (gg.aOnce()) {
            currentState = States.CHANGING_ANGLE;
            Log.w("debug", "pressed");
        }
        switch (currentState){
            case CHANGING_ANGLE:
                LLResult result = lm.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        double heading_error = sign * result.getTx();
                        tx = result.getTx();
                        ty = result.getTy();
                        telemetry.addData("heading error", heading_error);
                        if (heading_error < 0) {
                            power = kP * heading_error - min_command;
                        } else {
                            power = kP * heading_error + min_command;
                        }
                        target = 35.11*ty + 1026 - overShootVariable;
                        telemetry.addData("power",power);
                        if (Math.abs(heading_error) < threeshold || entered) {
                            degrees = rectangleOrientation(result);
//                                if(Math.abs(degrees) > 140) {
//                                    robot.arm.clawSubsystem.rotateState = Claw.RotateMode.VERTICAL;
//                                }else {
//                                    robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//                                }

                            telemetry.addData("target",target);
                            entered = true;

                            dt.setPowers(0, 0, 0);




                            if(counterV + counterH > checkLimit) {
                                if(counterV>counterH) vertical = true;
                                else vertical = false;
                                robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
                                counterH = 0;
                                counterV = 0;
                                currentState = States.WAIT_FOR_CLAW;
                                entered = false;
                            }
                        }else {
                            dt.setPowers(0, 0, power);
                        }


                    }
                }
                break;
            case WAIT_FOR_CLAW:
                if(timerMidClaw == null) {
                    timerMidClaw = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                }
                if(timerMidClaw.time() >= midTime) {
                    timerMidClaw = null;
                    robot.arm.changeExtension(target);
                    currentState = States.EXTENDING;
                }
                break;
            case EXTENDING:

                if(robot.arm.extensionSubsystem.mode == Extension.MODE.IDLE || robot.arm.extensionSubsystem.mode == Extension.MODE.RAW_POWER) {
                    robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;

                    if(vertical){
                        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.VERTICAL;
                    }
                    if(!revColor.isRed()) robot.arm.extensionSubsystem.changeRawPower(powerExtension);
                    if(revColor.isRed()) {
                        robot.arm.extensionSubsystem.changeRawPower(0);
                        robot.arm.extensionSubsystem.mode = Extension.MODE.IDLE;
                        currentState = States.STOP;
                    }
                }

                break;
            case STOP:
                vertical = false;
                robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
                currentState = States.IDLE;
                break;
            case IDLE:
                break;
        }
        telemetry.addData("blue", revColor.isBlue());
        telemetry.addData("yellow", revColor.isYellow());
        telemetry.addData("red", revColor.isRed());
        telemetry.addData("tookIt", revColor.tookit());
        telemetry.addData("tx",tx);
        telemetry.addData("ty",ty);
        telemetry.addData("pid", pid);
        telemetry.addData("entered",entered);
        if(timerMidClaw != null) {
            telemetry.addData("timer",timerMidClaw.time());
        }
        telemetry.addData("sum ",counterH + counterV);
        telemetry.addData("currentState",currentState);
        telemetry.addData("extension encoder",robot.arm.extensionSubsystem.currentPos);
        telemetry.addData("extension arm",robot.arm.extensionSubsystem.mode);
        telemetry.addData("extension power",robot.arm.extensionSubsystem.raw_power);
        telemetry.addData("currentState",robot.arm.currentState);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }



}
