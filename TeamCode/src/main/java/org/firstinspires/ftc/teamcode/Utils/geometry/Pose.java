package org.firstinspires.ftc.teamcode.Utils.geometry;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Locale;

public class Pose extends Point {

    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = AngleUnit.normalizeRadians(heading);
    }

    public Pose(Point p, double heading) {
        this(p.x, p.y, heading);
    }

    public Pose(Vector2D vec, double heading) {
        this(vec.x, vec.y, heading);
    }

    public Pose(Pose2d pose2d) {
        this(pose2d.position.x, pose2d.position.y, pose2d.heading.toDouble());
    }

    public Pose(Pose2D pose2d) {
        this(pose2d.getX(DistanceUnit.INCH), pose2d.getY(DistanceUnit.INCH), pose2d.getHeading(AngleUnit.RADIANS));
    }

    public Pose2D getFtc() {
        return new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.RADIANS,heading);

    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(AprilTagPoseFtc ftcPose) {
        this.heading = Math.toRadians(-ftcPose.yaw);
        this.x = ftcPose.x * Math.cos(heading) - ftcPose.y * Math.sin(heading);
        this.y = ftcPose.x * Math.sin(heading) + ftcPose.y * Math.cos(heading);
    }

    public void set(Pose other) {
        this.x = other.x;
        this.y = other.y;
        this.heading = other.heading;
    }

    public Pose add(Pose other) {
        return new Pose(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose subtract(Pose other) {
        return new Pose(this.x - other.x, this.y - other.y, AngleUnit.normalizeRadians(this.heading - other.heading));
    }

    public Pose divide(Pose other) {
        return new Pose(this.x / other.x, this.y / other.y, this.heading / other.heading);
    }

    public Pose scale(double scalar){
        return new Pose(this.x * scalar, this.y * scalar, this.heading * scalar);
    }

    public Pose subt(Pose other) {
        return new Pose(x - other.x, y - other.y, heading - other.heading);
    }

    public Vector2D toVec2D() {
        return new Vector2D(x, y);
    }

    public Pose computeFtcPose(Pose2D pose) {
        Pose ftcPose = new Pose(
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS)
        );
        return ftcPose;
    }

    public void copyFromPose2D(Pose2d pose2d) {
        this.x = pose2d.position.x;
        this.y = pose2d.position.y;
        this.heading = pose2d.heading.toDouble();
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "%.2f %.2f %.3f", x, y, heading);
    }
}