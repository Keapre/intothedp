//package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
//
//import android.util.Log;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.Utils.geometry.Path;
//import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;
//
//import java.util.ArrayList;
//
//@Config
//public class P2Pdrive extends Drive{
//
//    private ElapsedTime timer;
//    private ElapsedTime stable;
//
//    public static double STABLE_MS = 50;
//    public static double DEAD_MS = 2500;
//
//    public static  double  MAX_TRANSLATIONAL_SPEED = 0.7;
//    public static  double  MAX_ROTATIONAL_SPEED = 0.5;
//    private final double X_GAIN = 1.40;
//
//    public Pose targetPose;
//    public Pose currentPose;
//
//    public static double xP = 0.107;
//    public static double xD = 0.011;
//
//    public static double yP = 0.107;
//    public static double yD = 0.011;
//
//    public static double hP = 1.7;
//    public static double hD = 0.035;
//
//    Path path = null;
//
//    public  PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
//    public  PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
//    public  PIDFController hController = new PIDFController(hP, 0.0, hD, 0);
//
//    public Pose2d targetPose2d;
//    public static double ALLOWED_TRANSLATIONAL_ERROR = 1;
//    public static double ALLOWED_HEADING_ERROR = 0.05;
//
//
//    public DriveMode driveMode = DriveMode.IDLE;
//    public P2Pdrive(HardwareMap hardwareMap, Pose2d pose, boolean isAuto) {
//        super(hardwareMap, pose, isAuto);
//        targetPose2d = pose;
//        targetPose = new Pose(targetPose2d);
//        driveMode = DriveMode.IDLE;
//    }
//
//    public void setTargetPose(Path path) {
//        xController.reset();
//        yController.reset();
//        hController.reset();
//        this.path = path;
//        targetPose = path.get_currentPose();
//        driveMode = DriveMode.GO_TO_TARGET;
//
//    }
//
//    public void setTargetPose(Pose2d targetPose) {
//        xController.reset();
//        yController.reset();
//        hController.reset();
//        ArrayList<Pose> temp = new ArrayList<>();
//        temp.add(new Pose(targetPose));
//        setTargetPose(new Path(temp));
//        driveMode = DriveMode.GO_TO_TARGET;
//    }
//
//    public enum DriveMode {
//        IDLE,
//        GO_TO_TARGET
//    }
//    double getMagnitude(Vector2d vector) {
//        return Math.hypot(vector.x, vector.y);
//    }
//    public boolean isFinishedLast(Pose currentPose) {
//        Pose delta = targetPose.subtract(currentPose);
//
//        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR
//                || Math.abs(delta.heading) > ALLOWED_HEADING_ERROR) {
//            stable.reset();
//        }
//
//        return timer.milliseconds() > DEAD_MS || stable.milliseconds() > STABLE_MS;
//    }
//    public boolean isFinishedTransition(Pose currentPose) {
//        Pose delta = targetPose.subtract(currentPose);
//
//        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR + 15
//                || Math.abs(delta.heading) > ALLOWED_HEADING_ERROR + 6) {
//            return false;
//        }
//        return true;
//    }
//
//
//    void goToTarget() {
//
//        if (timer == null) timer = new ElapsedTime();
//        if (stable == null) stable = new ElapsedTime();
//         PoseVelocity2d vel = updatePoseEstimate();
//        currentPose = new Pose(pose.position.x, pose.position.y, pose.heading.toDouble());
//        if(!path.isLast()) {
//            MAX_TRANSLATIONAL_SPEED = 0.95;
//            MAX_ROTATIONAL_SPEED = 0.75;
//            //transition
//            if(isFinishedTransition(currentPose)) {
//                timer = null;
//                stable = null;
//
//                targetPose = path.next();
//                return;
//            }
//        }
//        if(path.isLast()) {
//            MAX_TRANSLATIONAL_SPEED = 0.6;//0.75
//            MAX_ROTATIONAL_SPEED = 0.45;//0.6
//            if(isFinishedLast(currentPose)) {
//                timer = null;
//                stable = null;
//                xController.reset();
//                yController.reset();
//                hController.reset();
//                setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//                driveMode = DriveMode.IDLE;
//            }
//        }
//
//
////        xController.setPIDF(xP, 0.0, xD, 0);
////        yController.setPIDF(yP, 0.0, yD, 0);
////        hController.setPIDF(hP, 0.0, hD, 0);
//
//        if(targetPose.heading - currentPose.heading > Math.PI) targetPose.heading -= 2 * Math.PI;
//        if(targetPose.heading - currentPose.heading < -Math.PI) targetPose.heading += 2 * Math.PI;
//
//        double xPower = xController.calculate(currentPose.x, targetPose.x);
//        double yPower = yController.calculate(currentPose.y, targetPose.y);
//        double hPower = hController.calculate(currentPose.heading, targetPose.heading);
//
//        Log.w("Robot power","xPower" + xPower);
//        Log.w("Robot power","yPower" + yPower);
//        Log.w("Robot power","hPower" + hPower);
//        double x_rotated = xPower * Math.cos(-currentPose.heading) - yPower * Math.sin(-currentPose.heading);
//        double y_rotated = xPower * Math.sin(-currentPose.heading) + yPower * Math.cos(-currentPose.heading);
//
//        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
//        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
//        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);
//
//        Log.w("Robot power","x_rotated" + x_rotated);
//        Log.w("Robot power","y_rotated" + y_rotated);
//        Log.w("Robot power","h" + hPower);
//
//        Log.w("Robot power", "target pose"+ String.valueOf(targetPose));
//        Log.w("Robot power", "target pose"+ String.valueOf(currentPose));
//        Log.w("Robot power", "state"+ driveMode);
//        setDrivePowers(new PoseVelocity2d(new Vector2d(x_rotated,y_rotated),hPower));
//
//    }
//
//    public void update() {
//        if (isAuto) {
//            switch (driveMode) {
//                case IDLE:
//                    setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
//                    break;
//                case GO_TO_TARGET:
//                    goToTarget();
//                    break;
//            }
//        }
//    }
//
//
//}