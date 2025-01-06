package org.firstinspires.ftc.teamcode.Utils.messages;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class Drawing {
    private Drawing() {}


    public static void drawRobot(Canvas c, Pose2d t,boolean currentPose) {
        final double ROBOT_RADIUS = 9;
        if(currentPose) {
            c.setStroke("blue");
        }else {
            c.setStroke("red");
        }
        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}