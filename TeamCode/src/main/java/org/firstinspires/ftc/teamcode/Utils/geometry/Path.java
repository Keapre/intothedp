package org.firstinspires.ftc.teamcode.Utils.geometry;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.List;

/**
 * Represents a list of Pose2d waypoints that the robot should follow in sequence.
 */
public class Path {
    private List<Pose2d> points;
    private int index = 0;

    public Path(List<Pose2d> points) {
        this.points = points;
    }

    /** Reset to start at the first waypoint. */
    public void reset() {
        index = 0;
    }

    /** Returns the current target Pose2d or null if the path is complete. */
    public Pose2d getCurrentTarget() {
        if (isComplete()) return null;
        return points.get(index);
    }

    /** Move to the next waypoint in the list. */
    public void nextPoint() {
        index++;
    }

    public boolean isTransition() {
        return index != points.size() - 1;
    }

    /** Returns true if we have visited all waypoints. */
    public boolean isComplete() {
        return index >= points.size();
    }

    public int getIndex() {
        return index;
    }

    public int size() {
        return points.size();
    }
}
