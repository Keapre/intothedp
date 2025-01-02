package org.firstinspires.ftc.teamcode.Utils.Control;

import org.firstinspires.ftc.teamcode.Utils.Utils;

public class SquidController {
    public double p;
    public double i;
    public double d;
    public SquidController(double P, double I, double D){
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

        double result = proportion + integral + derivative;
        result = Math.sqrt(Math.abs(result)) * Math.signum(result);

        return Utils.minMaxClip(result, min, max);
    }

    public void updatePID(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}
