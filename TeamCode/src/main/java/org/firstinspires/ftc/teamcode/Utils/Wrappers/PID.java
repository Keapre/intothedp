package org.firstinspires.ftc.teamcode.Utils.Wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Utils;

// https://www.ctrlaltftc.com/the-pid-controller
@Config
public class PID {
    public double p;
    public double i;
    public double d;
    public PID(double P, double I, double D){
        p=P;
        i=I;
        d=D;
    }
    double integral = 0;
    long lastLoopTime = System.nanoTime();
    double lastError = 0;
    int counter = 0;
    double loopTime = 0.0;

    public void reset() {
        integral = 0;
        lastError = 0;
        counter = 0;
        lastLoopTime = System.nanoTime();
    }

    public double update(double error, double min, double max){
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10000000;
        }

        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime)/1000000000.0;
        lastLoopTime = currentTime; // lastLoopTime's start time

        double proportion = p * error;
        integral += error * i * loopTime;
        double derivative = d * (error - lastError)/loopTime;

        lastError = error;
        counter ++;

        return Utils.minMaxClip(proportion + integral + derivative, min, max);
    }

    public void updatePID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}