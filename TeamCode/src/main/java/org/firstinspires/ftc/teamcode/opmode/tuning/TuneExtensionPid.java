package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@Config
@TeleOp(name = "tuning extension pid",group = "tuning")
public class TuneExtensionPid extends LinearOpMode {
    Robot robot = null;
    public static double target = 0;
    public double prev_target = target;
    public enum Mode{
        PID,
        IDLE
    }

    Mode mode = Mode.IDLE;
    GamePadController gg;
    /*
    tune first Kf si dupa coeficienti

     */
    long lastLoopFinish = System.currentTimeMillis();
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = false;
        robot = new Robot(this,new Pose2d(0,0,0));
        robot.drive.IS_DISABLED = true;
        gg = new GamePadController(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        robot.start();

        while(opModeIsActive()) {
            if(prev_target!=target) {
                robot.arm.changeExtension(target);
            }
            prev_target = target;

            telemetry.addData("target",target);
            telemetry.addData("currentPos",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("arm state",robot.arm.currentState);
            telemetry.addData("extension state",robot.arm.extensionSubsystem.mode);
            telemetry.addData("ff",robot.arm.extensionSubsystem.ff);
            telemetry.addData("angle",robot.arm.extensionSubsystem.angle);
            telemetry.addData("power",robot.arm.extensionSubsystem.power);
            telemetry.addData("Sample Rate (Hz) ",1/((double)(System.currentTimeMillis() - lastLoopFinish)/1000.0));
            lastLoopFinish = System.currentTimeMillis();
            telemetry.addData("pos",robot.arm.extensionSubsystem.getTrueCurrentPosition());
            telemetry.update();
        }
    }
}
