package org.firstinspires.ftc.teamcode.Utils.Wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
public class WrapEncoder {

    private final double rangeLow,rangeHigh;

    AnalogInput encoder = null;
    private double lastValue = 0.0;
    private int numRotations = 0;
    public static double divider = 2.0;//adjust dupa loop time


    public WrapEncoder(double rangeLow, double rangeHigh,AnalogInput e){
        this.rangeLow = rangeLow;
        this.rangeHigh = rangeHigh;
        encoder = e;
    }


    public double getValue() {
        double value = encoder.getVoltage() / 3.3 * (rangeHigh - rangeLow);
        return value + numRotations * (rangeHigh - rangeLow);
    }


    public void update(){
        double value = encoder.getVoltage() / 3.3 * (rangeHigh - rangeLow);
        if(Math.abs(value - lastValue) > (rangeHigh - rangeLow) / divider){
            numRotations += (value > lastValue) ? -1 : 1;
        }

        lastValue = value;
    }

}
