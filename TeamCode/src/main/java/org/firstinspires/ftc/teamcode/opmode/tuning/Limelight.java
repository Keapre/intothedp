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

        double xA, yA, xB, yB,xC,yC,xD,yD;
        int size = targetCorners.size();
        if(size < 4) {
            return 0;
        }
        double minX=1e9,maxX=-1e9,minY=1e9,maxY=-1e9;
//        xA = targetCorners.get(0).get(0);
//        yA = targetCorners.get(0).get(1);

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
            telemetry.addData("Orientation","Horizontal");
        }else {
            telemetry.addData("Orientation","Vertical");
        }
        return 0;
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
