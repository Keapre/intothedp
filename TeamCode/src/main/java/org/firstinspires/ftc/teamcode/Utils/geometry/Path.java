package org.firstinspires.ftc.teamcode.Utils.geometry;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.ArrayList;

public class Path {
    ArrayList<Pose> poses = new ArrayList<>();
    int get_currentPoseIndex = 0;
    public Path(ArrayList<Pose> poses) {
        this.poses = poses;
        get_currentPoseIndex = 0;
    }

    public Pose get_currentPose(){
        return poses.get(get_currentPoseIndex);
    }

    public Pose get_nextPose(){
        return poses.get(get_currentPoseIndex + 1);
    }

    public void incrementPoseIndex(){
        get_currentPoseIndex++;
    }

    public Pose next() {
        if(!isLast()) incrementPoseIndex();
        return get_currentPose();
    }
    public boolean isLast() {
        return get_currentPoseIndex == poses.size() - 1;
    }

    public boolean isFinished() {
        return get_currentPoseIndex == poses.size();
    }


}
