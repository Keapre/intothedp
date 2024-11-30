package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utils.MecanumUtil;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.Utils.messages.PoseMessage;


public class Drive extends MecanumDrive{
    public static class Params {
        public double xOffset = -115;
        public double yOffset = -2.5;


        public double encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;

        public GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        public GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }
    public static Params PARAMS = new Params();
    public GoBildaPinpointDriverRR pinpoint;
    public static boolean usePin = false;
    private Pose2d lastPinpointPose = pose;
    public Drive(HardwareMap hardwareMap, Pose2d pose, boolean isAuto) {
        super(hardwareMap, pose, isAuto);
        FlightRecorder.write("PINPOINT_PARAMS",PARAMS);
        pinpoint = hardwareMap.get(GoBildaPinpointDriverRR.class,"odo");

        // RR localizer note: don't love this conversion (change driver?)
        pinpoint.setOffsets(PARAMS.xOffset, PARAMS.yOffset);


        pinpoint.setEncoderResolution(PARAMS.encoderResolution);

        pinpoint.setEncoderDirections(PARAMS.xDirection, PARAMS.yDirection);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
        // wait for pinpoint to finish calibrating
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        pinpoint.setPosition(pose);
    }
    public void setMotorPowersFromGamepad(GamePadController gg, double scale, boolean reverseFront, boolean customCurve) {
        MecanumUtil.Motion motion;

        if(gg.leftStickButtonOnce()){
            slow_mode = !slow_mode;
        }
        if(gg.rightStickButtonOnce()) {
            fieldCentric = !fieldCentric;
        }
        double left_stick_x = gg.left_stick_x;
        double left_stick_y = gg.left_stick_y;
        double right_stick_x = gg.right_stick_x;
        double right_stick_y = gg.right_stick_y;

        if(slow_mode) {
            left_stick_x*=slowMode;
            right_stick_y*=slowMode;
            right_stick_x*=slowMode;
            right_stick_y*=slowMode;
        }else {
            right_stick_x*=rotateNormal;
            right_stick_y*=rotateNormal;
        }
        motion = MecanumUtil.joystickToMotion(left_stick_x, left_stick_y,
                right_stick_x, right_stick_y, reverseFront, customCurve);


        updatePoseEstimate();
        if (fieldCentric) {
            motion = motion.toFieldCentricMotion(pose.heading.toDouble());
        }

        MecanumUtil.Wheels wh = MecanumUtil.motionToWheelsFullSpeed(motion).scaleWheelPower(scale); // Use full forward speed on 19:1 motors
/*        motorPowers[0] = ffMotor.compute(wh.frontLeft,PARAMS.maxProfileAccel);
        motorPowers[1] = ffMotor.compute(wh.backLeft,PARAMS.maxProfileAccel);
        motorPowers[2] = ffMotor.compute(wh.backRight,PARAMS.maxProfileAccel);
        motorPowers[3] = ffMotor.compute(wh.frontRight,PARAMS.maxProfileAccel);*/
        motorPowers[0] = wh.frontLeft;
        motorPowers[1] = wh.backLeft;
        motorPowers[2] = wh.backRight;
        motorPowers[3] = wh.frontRight;
        setMotorPowers(motorPowers[0],motorPowers[1],motorPowers[2],motorPowers[3]);
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastPinpointPose != pose) {

            pinpoint.setPosition(pose);
        }
        usePin = fieldCentric;
        if(usePin){
            pinpoint.update();
            pose = pinpoint.getPositionRR();
        }

        lastPinpointPose = pose;

        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        FlightRecorder.write("ESTIMATED_POSE", new PoseMessage(pose));
        FlightRecorder.write("PINPOINT_RAW_POSE",new FTCPoseMessage(pinpoint.getPosition()));
        FlightRecorder.write("PINPOINT_STATUS",pinpoint.getDeviceStatus());

        return null;
    }

    public static final class FTCPoseMessage {
        public long timestamp;
        public double x;
        public double y;
        public double heading;

        public FTCPoseMessage(Pose2D pose) {
            this.timestamp = System.nanoTime();
            this.x = pose.getX(DistanceUnit.INCH);
            this.y = pose.getY(DistanceUnit.INCH);
            this.heading = pose.getHeading(AngleUnit.RADIANS);
        }
    }

}