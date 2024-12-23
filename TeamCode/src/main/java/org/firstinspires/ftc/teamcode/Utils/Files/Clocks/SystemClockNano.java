package org.firstinspires.ftc.teamcode.Utils.Files.Clocks;

public class SystemClockNano implements Clock {
    private long startTime;
    public SystemClockNano(){
        startTime = System.nanoTime();
    }

    public long getCurrentTime(){
        return System.nanoTime();
    }
    public double getTimePassed(){
        return (getCurrentTime() - startTime);
    }

}
