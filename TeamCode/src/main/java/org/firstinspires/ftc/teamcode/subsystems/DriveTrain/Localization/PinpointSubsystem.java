package org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Localization;

import android.bluetooth.BluetoothClass;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Localization.LocalizerConstants;

import java.util.LinkedList;
import java.util.Optional;

@Config

public class PinpointSubsystem {
    GoBildaPinpointDriver pinpoint;
    Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    GoBildaPinpointDriver.DeviceStatus deviceStatus = GoBildaPinpointDriver.DeviceStatus.NOT_READY;

    @Deprecated
    public static double yawScalar = 1;
    public static boolean flipX = true;
    public static boolean flipY = true;
    public static double xEncOffset = LocalizerConstants.xOffset;
    public static double yEncOffset = LocalizerConstants.yOffset;

    private Pose2D lastGoodPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

    public PinpointSubsystem(HardwareMap hMap, Pose2d startingPose)
        {
            pinpoint = hMap.get(GoBildaPinpointDriver.class, "pinpoint");

            pinpoint.initialize();

            //pinpoint.setYawScalar(yawScalar);
            pinpoint.recalibrateIMU();

            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
            pinpoint.setEncoderDirections(
                    flipX ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    flipY ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            pinpoint.setOffsets(xEncOffset, yEncOffset);
            //reset();
            deviceStatus = pinpoint.getDeviceStatus();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startingPose.getX(), startingPose.getY(), AngleUnit.RADIANS, startingPose.getHeading()));
        }

        public void update () {
            pinpoint.update();
            if (Double.isNaN(pinpoint.getPosX()) || Double.isNaN(pinpoint.getPosY()) ||
                    (pinpoint.getPosX() == 0.0
                            && pinpoint.getPosY() == 0.0
                            && pinpoint.getHeading() == 0.0
                            && pinpoint.getVelX() == 0.0)) {
                pose = lastGoodPose;
                Log.i("%11", "pinpoint NaN value");
            } else {
                pose = pinpoint.getPosition();
                lastGoodPose = pose;
            }

            //Log.i("posex", "" + pose.getX(DistanceUnit.INCH));

            if (pinpoint.getDeviceStatus() != deviceStatus) {
                Log.i("%Pinpoint Status Change", pinpoint.getDeviceStatus().toString());
                deviceStatus = pinpoint.getDeviceStatus();
            }
        }

        private Pose2D getSDKPose () {
            return pose;
        }

        public int[] getEncoderCounts () {
            return new int[]{pinpoint.getEncoderX(), pinpoint.getEncoderY()};
        }

        public Pose2d getPose () {
            return new Pose2d(getSDKPose().getX(DistanceUnit.INCH),
                    getSDKPose().getY(DistanceUnit.INCH),
                    Rotation2d.fromDegrees(getSDKPose().getHeading(AngleUnit.RADIANS)));
        }

        public Pose2d getVelocity () {
            Pose2D velocity = pinpoint.getVelocity();
            return new Pose2d(velocity.getX(DistanceUnit.INCH),
                    velocity.getY(DistanceUnit.INCH),
                    Rotation2d.fromDegrees(velocity.getHeading(AngleUnit.RADIANS)));
        }

        public void reset () {
            pinpoint.resetPosAndIMU();
            lastGoodPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
        }

        /**
         * @param x In inches.
         * @param y In inches.
         */
        public void setPosition ( double x, double y){
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS)));
            lastGoodPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS));
        }
        public boolean isDoneCalibration () {

            return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        }


        /**
         * Warning - will completely break position!!
         */
        public void resetYaw () {
            pinpoint.setPosition(new Pose2D(DistanceUnit.MM, pinpoint.getPosX(), pinpoint.getPosY(), AngleUnit.RADIANS, 0));
        }
    }