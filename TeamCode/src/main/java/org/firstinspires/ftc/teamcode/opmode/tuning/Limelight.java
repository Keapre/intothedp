package org.firstinspires.ftc.teamcode.opmode.tuning;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Test Detection")
public class Limelight extends LinearOpMode {

    Limelight3A lm;
    public double rectangleOrientation(LLResult result) {
        List<List<Double>> targetCorners = result.getColorResults().get(0).getTargetCorners();


        double xA, yA, xB, yB,xC,yC;
        xA = targetCorners.get(0).get(0);
        yA = targetCorners.get(0).get(1);

        telemetry.addData("corners",targetCorners.toString());
        xB = targetCorners.get(1).get(0);
        yB = targetCorners.get(1).get(1);

        xC = targetCorners.get(2).get(0);
        yC = targetCorners.get(2).get(1);
        double x = xB - xA;
        double y = yC - yA;
        double angle = Math.atan2(y,x);

        double degreesSal = Math.toDegrees(angle);

        double witdth = Math.abs(xB-xA);
        double length = Math.abs(yA-yC);
        if(witdth > length) {
            telemetry.addData("Orientation","Horizontal");
        }else {
            telemetry.addData("Orientation","Vertical");
        }
        Log.w("debug", "angle: " + degreesSal);
        return degreesSal;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        lm = hardwareMap.get(Limelight3A.class,"limelight");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        lm.start();
        while(opModeIsActive()) {
            LLResult result = lm.getLatestResult();

            if (result != null) {
                if (result.isValid()) {
                    telemetry.addData("tx",result.getTx());
                    telemetry.addData("ty",result.getTy());
                    telemetry.addData("angle",rectangleOrientation(result));
                }
            }
            telemetry.update();
        }
    }
}
