package org.firstinspires.ftc.teamcode.Utils.Caching;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utils.Utils;

public class CachingDcMotorEx extends CachingDcMotor implements DcMotorEx {
    private DcMotorEx delegate;
    double lastPower = 0;
    public double power = 0;

    private double minPowerToOvercomeStaticFriction = 0.0;
    public static final int SWITCH_FROM_STATIC_TO_KINETIC_FRICTION = 75;
    private double minPowerToOvercomeKineticFriction = 0.0;
    private long lastZeroTime = 0;
    public CachingDcMotorEx(DcMotorEx dcMotorEx,double tolerance) {
        super(dcMotorEx,tolerance);
        delegate = dcMotorEx;
    }

    @Override
    public void setMotorEnable() {
        delegate.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        delegate.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return delegate.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        delegate.setVelocity(angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        delegate.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return delegate.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return delegate.getVelocity(unit);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        delegate.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        delegate.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        delegate.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        delegate.setPositionPIDFCoefficients(p);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return delegate.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return delegate.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        delegate.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return delegate.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return delegate.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return delegate.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        delegate.setCurrentAlert(current, unit);
    }
    double k = 0.7; // 0.5
    public void setTargetPowerSmooth(double power,double voltage) {
        if (lastPower == 0){
            lastZeroTime = System.currentTimeMillis();
        }
        if (power == 0) {
            this.power = 0;
            return;
        }
        power = Utils.minMaxClip(power, -1.0, 1.0);
        double m = (System.currentTimeMillis() > SWITCH_FROM_STATIC_TO_KINETIC_FRICTION + lastZeroTime ? minPowerToOvercomeKineticFriction : minPowerToOvercomeStaticFriction) * (13.5/voltage);
        power *= 1-m;
        power = power + m * Math.signum(power);
        this.power = power*k + this.lastPower*(1-k);
    }
    @Override
    public boolean isOverCurrent() {
        return delegate.isOverCurrent();
    }
}