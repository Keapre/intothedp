package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@Photon
@TeleOp(name = "eric e smech")
public class EricTeleOp extends OpMode {
    Robot robot;
    GamePadController gg;

    long lastLoopFinish;

    @Override
    public void init() {
        gg = new GamePadController(gamepad1);
        robot = new Robot(this,false);
        lastLoopFinish = System.currentTimeMillis();
    }


    @Override
    public void start() {
        robot.start();
    }
    private boolean isManualControlActivated() {
        // Implement logic to detect manual control activation
        return Math.abs(gg.left_trigger) > 0.1 || Math.abs(gg.right_trigger) > 0.1 || gg.aOnce() || gg.rightBumper() || gg.leftBumper();
    }

    private boolean manualControlDeactivated() {
        // Define condition to exit manual control
        return Math.abs(gg.left_stick_y) < 0.1 && Math.abs(gg.right_stick_y) < 0.1 && !gg.aOnce() && !gg.leftBumper() && !gg.rightBumper();
    }


    private static String formatResults(MovingStatistics statistics) {
        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
                statistics.getMean() * 1000,
                statistics.getStandardDeviation() * 1000,
                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
    }

    private void addStatistics() {
        telemetry.addData("Top 250", formatResults(robot.top250));
        telemetry.addData("Top 100", formatResults(robot.top100));
        telemetry.addData("Top 10", formatResults(robot.top10));
    }
    @Override
    public void loop()
    {
        gg.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        updateDrive();
       // updateArm();
        updateTelemetry();
    }

//    public void updateArm() {
//
//        if(isManualControlActivated()) {
//            robot.arm.manualControl = true;
//            robot.arm.handleManualControl(gg);
//        }else if(manualControlDeactivated()) {
//            robot.arm.manualControl = false;
//        }
//        if(gg.bOnce()) {
//            if(robot.arm.targetState == new INTAKING() && (robot.arm.getCurrentState() == Arm.FSMState.OPERATION_COMPLETE
//                    || robot.arm.getCurrentState() == Arm.FSMState.IDLE)) {
//                robot.arm.setTargetState(new DEFAUlT());
//            }else if((robot.arm.getCurrentState() == Arm.FSMState.OPERATION_COMPLETE
//                    || robot.arm.getCurrentState() == Arm.FSMState.IDLE)) {
//                robot.arm.setTargetState(new INTAKING());
//            }
//        }
//        if (gg.yOnce()) {
//            if(robot.arm.targetState == new HIGHBASKET() && (robot.arm.getCurrentState() == Arm.FSMState.OPERATION_COMPLETE
//                    || robot.arm.getCurrentState() == Arm.FSMState.IDLE)) {
//                robot.arm.setTargetState(new DEFAUlT());
//            }else if((robot.arm.getCurrentState() == Arm.FSMState.OPERATION_COMPLETE
//                    || robot.arm.getCurrentState() == Arm.FSMState.IDLE)) {
//                robot.arm.setTargetState(new HIGHBASKET());
//            }
//        }
//        if(gg.xOnce()) {
//            if(robot.arm.targetState == new SPECIMEN() && (robot.arm.getCurrentState() == Arm.FSMState.OPERATION_COMPLETE
//                    || robot.arm.getCurrentState() == Arm.FSMState.IDLE)) {
//                robot.arm.setTargetState(new DEFAUlT());
//            }else if((robot.arm.getCurrentState() == Arm.FSMState.OPERATION_COMPLETE
//                    || robot.arm.getCurrentState() == Arm.FSMState.IDLE)) {
//                robot.arm.setTargetState(new SPECIMEN());
//            }
//        }
//    }
    public void updateDrive() {
        robot.drive.setMotorPowersFromGamepad(gg,1.0,false,true);
    }
    public void updateTelemetry() {
//        telemetry.addLine("--------DRIVETRAIN------");
//        telemetry.addData("Powers",robot.mecanum.motorPowers);
//        telemetry.addLine();
//        telemetry.addLine("---------------ARM--------------");
//        telemetry.addData("ARM STATE",robot.arm.targetState);
//        telemetry.addData("ARM CURRENT FSM",robot.arm.currentState);
//        telemetry.addData("Extension pos",robot.arm.extensionSubsystem.currentPos);
//        telemetry.addData("Pith pos",robot.arm.pitchSubsystem.calculateAngle());
//        telemetry.addLine();
        telemetry.addData("Sample Rate (Hz) ",1/((double)(System.currentTimeMillis() - lastLoopFinish)/1000.0));
        addStatistics();
        telemetry.update();
        lastLoopFinish = System.currentTimeMillis();
    }
}
