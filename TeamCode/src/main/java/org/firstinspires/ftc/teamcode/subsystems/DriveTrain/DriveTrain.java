package org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.MecanumUtil;
import org.firstinspires.ftc.teamcode.Utils.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Filters.SlewRateFilter;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.PID;
import org.firstinspires.ftc.teamcode.Utils.Control.SquidController;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.TelemetryUtil;
import org.firstinspires.ftc.teamcode.Utils.geometry.Path;
import org.firstinspires.ftc.teamcode.Utils.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.Utils.messages.Drawing;

/*
IMPROVEMENTS
MAYBE ADD SECODARY SQUID CONTROLLER for more precise control
TEST MORE THE PINPOINT DRIVER
SI SA TESTEZEZ mai multe functii pt gamepad sigmoid or cubic

 */

@Config
public class DriveTrain implements Subsystem {


    private Robot robot;

    public static boolean useEquation = true;

    public boolean slow_mode = false;
    public static boolean useFieldCentric = true;
    public enum STATE {
        IDLE,
        GOING_TO_POINT,
        DRIVE,
        TURN,
        FINE_CONTROL,
        PATH_FOLLOW,
        GLIDE
    }

    public STATE state = STATE.IDLE,lastState = STATE.IDLE;
    public boolean IS_DISABLED = false;
    private CachingDcMotorEx leftFront, leftBack, rightBack, rightFront;

    public static double pinPointxOffset = 128.50, pinPointyOffset = 5.5; // ar trb sa fie in mm //odometria din stanga (x)
    GoBildaPinpointDriverRR pinpoint;

    boolean IS_AUTO = false;
    Pose2d lastPinPoint = new Pose2d(0,0,0);
    Pose2d pose = new Pose2d(0,0,0);
    Pose2d target = new Pose2d(0,0,0);
    PoseVelocity2d speed = new PoseVelocity2d(new Vector2d(0,0),0);

    public double xSlowModeMultipler = 0.6,ySlowModeMultiplier = 0.6,hSlowModeMultiplier = 0.5;
    public double xNormalModeMultipler = 0.9,ySNormalModeMultiplier = 0.9,hNormalModeMultiplier = 0.9;
    ElapsedTime timer = null,stable = null;

    /* PIDS */
    //TODO: tune this
    public static double xkP = 0.1,xD = 0.1;

    public static double filterParameter= 0.3;
    SlewRateFilter filterX = new SlewRateFilter(filterParameter);
    SlewRateFilter filterY = new SlewRateFilter(filterParameter);
    SlewRateFilter filterH = new SlewRateFilter(filterParameter);
    public static boolean use_filter = false;

    public static double ykP = 0.1,ykD = 0.1;
    public static double hkP = 0.1,hkD = 0.1;
    public static PID xPID = new PID(xkP,0,xD);
    public static PID yPID = new PID(ykP,0,ykD);
    public static PID hPID = new PID(hkP,0,hkD);

    public static  double kS = 0.0255965909; // TODO: tune this
    PoseVelocity2d powerVector = new PoseVelocity2d(new Vector2d(0,0),0);
    private VoltageSensor voltageSensor;

    public static double decelX = 20,deccelY = 20; // TODO: tune this

    public static double xMultiplier = 1.1; // TODO: tune this
    public static boolean use_gliding = true;

    public static double xThreeshold = 2,yThreeshold = 2,hThreeshold = 0.2; //TODO:maybe tune this also

    Path path = null;
    boolean fine_stop = false,stop = true;
    double xError = 0,yError = 0,hError = 0;

    double max_speed = 1;

    public DriveTrain(HardwareMap hw, Pose2d startingPose,boolean auto,Robot robot) {
        this.robot = robot;
        leftFront =  new CachingDcMotorEx(hw.get(DcMotorEx.class,"leftFront"),0);
        leftBack =  new CachingDcMotorEx(hw.get(DcMotorEx.class,"leftBack"),0);
        rightBack =  new CachingDcMotorEx(hw.get(DcMotorEx.class,"rightBack"),0.0);
        rightFront =  new CachingDcMotorEx(hw.get(DcMotorEx.class,"rightFront"),0);



        IS_AUTO = Globals.IS_AUTO;
        initializeMotors();
        this.pose = startingPose;
        pinpoint = hw.get(GoBildaPinpointDriverRR.class,"odo");

        pinpoint.setOffsets(pinPointxOffset,pinPointyOffset);
        pinpoint.setEncoderResolution(GoBildaPinpointDriverRR.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        pinpoint.setPosition(pose);
        lastPinPoint = pose;
    }

    public void setMaxSpeed(double speed) {
        max_speed = speed;
    }

    public void setTarget(Pose2d target) {
        this.target = target;
    }

    public double getMaxSpeed() {
        return max_speed;
    }

    public void setCustomPowerVector(PoseVelocity2d power) {
        powerVector = new PoseVelocity2d(new Vector2d(power.linearVel.x,power.linearVel.y), power.angVel);
    }

    public PoseVelocity2d getPowerVector() {
        return powerVector;
    }
    public Pose2d getPose() {
        return pose;
    }
    void initializeMotors() {


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(IS_AUTO) {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }else {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void update() {
        if(IS_DISABLED) return;
        updateLocalization();


        switch (state) {
            case IDLE:
                if(lastState!=state) {
                    motorsOff();
                }
                break;
            case GOING_TO_POINT:
                calculateErrors();
                pidDrive();
                if(atTarget()) {
                    if(fine_stop) {
                        timer = null;
                        stable = null;
                        state = STATE.FINE_CONTROL;
                    } else {
                        state = STATE.IDLE;
                    }
                }
                break;
            case FINE_CONTROL:
                fineXPid.reset();
                fineYPid.reset();
                fineHPid.reset();
                fine_control();
                if(atTarget()) {
                    state = STATE.IDLE;
                }
                break;
            case PATH_FOLLOW:
                doPathFollowLogic();
            case DRIVE:
                break;
        }
        setPowerVector();
        updateTelemetry();
        lastState = state;
    }

    private void doPathFollowLogic() {
        if(path == null || path.isComplete()) {
            state = STATE.IDLE;
            return;
        }

        Pose2d currentWaypoint = path.getCurrentTarget();
        if(currentWaypoint == null) {
            state = STATE.IDLE;
            return;
        }

        this.target = currentWaypoint;
        if(path.isTransition()) {
            stop = false;
        }else {
            stop = true;
        }
        calculateErrors();
        pidDrive();


        if(atTarget()) {
            path.nextPoint();

            if(path.isComplete()) {
                if(fine_stop) {
                    state = STATE.FINE_CONTROL;
                } else {
                    state = STATE.IDLE;
                }
            } else {
                xPID.reset();
                yPID.reset();
                hPID.reset();
                timer = null;
                stable = null;
            }
        }
    }
    public void xSlowAdjust(double addon) {
        xSlowModeMultipler+=addon;
        xSlowModeMultipler = clamp(xSlowModeMultipler,0,1);
    }

    public void ySlowAdjust(double addon) {
        ySlowModeMultiplier+=addon;
        ySlowModeMultiplier = clamp(ySlowModeMultiplier,0,1);
    }

    public void hSlowAdjust(double addon) {
        hSlowModeMultiplier+=addon;
        hSlowModeMultiplier = clamp(hSlowModeMultiplier,0,1);
    }

    public void updateTelemetry() {

        TelemetryUtil.packet.put("Drivetrain State", state);

        TelemetryUtil.packet.put("xError", xError);
        TelemetryUtil.packet.put("yError", yError);
        TelemetryUtil.packet.put("turnError (deg)", Math.toDegrees(hError));

        TelemetryUtil.packet.put("maxPower", max_speed);

        Drawing.drawRobot(TelemetryUtil.packet.fieldOverlay(),pose,true);
//        TelemetryUtil.packet.fieldOverlay().setStroke("red");
//        TelemetryUtil.packet.fieldOverlay().strokeCircle(target.position.x, target.position.y, xThreeshold);

//        TelemetryUtil.packet.fieldOverlay().setStroke("blue");
//        TelemetryUtil.packet.fieldOverlay().strokeCircle(pose.position.x, pose.position.y, xThreeshold);
        Drawing.drawRobot(TelemetryUtil.packet.fieldOverlay(),target,false);

    }

    public void updateLocalization() {
        pinpoint.update();
        pose = pinpoint.getPositionRR();
        speed = pinpoint.getVelocityRR();
        lastPinPoint = pose;
    }

    public void calculateErrors() {
        double deltaX = (target.position.x - pose.position.x);
        double deltaY = (target.position.y - pose.position.y);

        xError = deltaX * Math.cos(-pose.heading.toDouble()) - deltaY * Math.sin(-pose.heading.toDouble());
        yError = deltaX * Math.sin(-pose.heading.toDouble()) + deltaY * Math.cos(-pose.heading.toDouble());
        hError = target.heading.toDouble() - pose.heading.toDouble();

        while(Math.abs(hError) > Math.PI ){
            hError -= Math.PI * 2 * Math.signum(hError);
        }
    }
    double FxkP = 0.1,FykP = 0.1,FhkP = 0.1; // TODO: tune this
    SquidController fineXPid = new SquidController(FxkP,0,0);
    SquidController fineYPid = new SquidController(FykP,0,0);
    SquidController fineHPid = new SquidController(FhkP,0,0);

    public void fine_control() {
        fineXPid.updatePID(FxkP,0,0);
        fineYPid.updatePID(FykP,0,0);
        fineHPid.updatePID(FhkP,0,0);

        double xPower = fineXPid.update(xError,-1,1);
        double yPower = fineYPid.update(yError,-1,1);
        double hPower = fineHPid.update(hError,-1,1);

        if(xError < 0.1) {
            xPower = 0;
        }
        if(yError < 0.1) {
            yPower = 0;
        }
        if(hError < 0.1) {
            hPower = 0;
        }
        powerVector = new PoseVelocity2d(new Vector2d(xPower,yPower),hPower);
    }
    public void setFinalAdjustment(boolean finalAdjustment) {
        this.fine_stop = finalAdjustment;
    }

    public void setStop(boolean stop) {
        this.stop = stop;
    }
    double fine_stable_time = 100,fine_max_time = 150;
    double normal_stable_time = 85,normal_max_time = 2000;
    public boolean atTarget() {
        if(timer == null) timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        if(stable == null) stable = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        if(fine_stop && state != STATE.GOING_TO_POINT) {
            if(!(Math.abs(xError) < xThreeshold / 2 && Math.abs(yError) < yThreeshold / 2 && Math.abs(hError) < Math.toRadians(hThreeshold))){
                stable.reset();
            }
            return stable.time() > fine_stable_time || timer.time() > fine_max_time;
        }

        if(!stop) {
            return (Math.abs(xError) < xThreeshold*4 && Math.abs(yError) < yThreeshold*4 && Math.abs(hError) < Math.toRadians(hThreeshold)*10) || timer.time() > normal_max_time;
        }

        if(!(Math.abs(xError) < xThreeshold && Math.abs(yError) < yThreeshold && Math.abs(hError) < Math.toRadians(hThreeshold))){
            stable.reset();
        }
        return stable.time() > normal_stable_time || timer.time() > normal_max_time;
    }

    public boolean atPointThresholds (double xThresh, double yThresh, double headingThresh) {
        return Math.abs(xError) < xThresh && Math.abs(yError) < yThresh && Math.abs(hError) < Math.toRadians(headingThresh);
    }

    void motorsOff() {
        powerVector = new PoseVelocity2d(new Vector2d(0,0),0);
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }





    public double strafe,forward,h;

    public double smoothControls(double value) {
        return 0.5*Math.tan(1.12*value);
    }

    double sigmoidScaling(double value) {
        return value / (1 + Math.abs(value));
    }

    double cubicScaling(double value) {
        return clamp(value * value * value,-1,1); // value^3
    }
    double deadzone = 0.05;
    public void setTargetPosition(Pose2d point, boolean finalAdjustment, boolean stop, double maxPower) {
        goToPoint(point, finalAdjustment, stop, maxPower);
    }

    private void setPath(Path path, boolean finalAdjustment, boolean stop, double maxPower) {
        this.path = path;
        this.path.reset();  // start at the first waypoint
        this.fine_stop = finalAdjustment;
        this.stop = stop;
        this.max_speed = Math.abs(maxPower);
        this.state = STATE.PATH_FOLLOW;

        // Reset PIDs, timers, etc. if needed
        xPID.reset();
        yPID.reset();
        hPID.reset();
        timer = null;
        stable = null;
    }
    // Overloaded method for a Path
    public void setTargetPosition(Path path, boolean finalAdjustment, boolean stop, double maxPower) {
        setPath(path, finalAdjustment, stop, maxPower);
    }
    double scaleInput(double value) {
        if (Math.abs(value) < deadzone) {
            return 0;
        }
        return sigmoidScaling(value);
    }

    public void drive(GamePadController gg) {
        state = STATE.DRIVE;
        max_speed = 1;
//        useEquation = false;
//        strafe = gg.left_stick_x;
//        forward = gg.left_stick_y;
//        h = gg.right_stick_x;
//        if(use_filter) {
//            strafe = filterX.calculate(strafe);
//            forward= filterY.calculate(forward);
//            h = filterH.calculate(h);
//        }
//
//        if(slow_mode) {
//            strafe*=xSlowModeMultipler;
//            forward*=ySlowModeMultiplier;
//            h*=hSlowModeMultiplier;
//        }
        MecanumUtil.Motion motion;
        double left_stick_x = -cubicScaling(gg.left_stick_x);
        double left_stick_y = -cubicScaling(gg.left_stick_y);
        double right_stick_x = -cubicScaling(gg.right_stick_x);
        double right_stick_y = gg.right_stick_y;



        if(slow_mode) {
            left_stick_y*=ySlowModeMultiplier;
            left_stick_x*=xSlowModeMultipler;
            right_stick_x*=hSlowModeMultiplier;
        }else {
            left_stick_y*=ySNormalModeMultiplier;
            left_stick_x*=xNormalModeMultipler;
            right_stick_x*=hNormalModeMultiplier;
        }
        left_stick_x*=xMultiplier;
        Vector2d translational = new Vector2d(left_stick_y,left_stick_x);
        if(useFieldCentric) {
            translational = pose.heading.inverse().times(translational);
        }
/*        motorPowers[0] = ffMotor.compute(wh.frontLeft,PARAMS.maxProfileAccel);
        motorPowers[1] = ffMotor.compute(wh.backLeft,PARAMS.maxProfileAccel);
        motorPowers[2] = ffMotor.compute(wh.backRight,PARAMS.maxProfileAccel);
        motorPowers[3] = ffMotor.compute(wh.frontRight,PARAMS.maxProfileAccel);*/
        powerVector = new PoseVelocity2d(translational,right_stick_x);
    }
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }
    public void setPowerVector() {
//        double[] powers = {
//                equationMotor(y+x+h),
//                equationMotor(y-x+h),
//                equationMotor(y+x-h),
//                equationMotor(y-x-h)
//        };
//
//        normalizeArray(powers);
        setDrivePowers(powerVector);
        //setMotorPowers(wheels.frontLeft,wheels.backLeft,wheels.backRight,wheels.frontRight);
        //setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }
    private double equationMotor(double rawPower) {


        double scaledKs = kS * Math.signum(rawPower);
        double finalPower = Utils.minMaxClip(scaledKs + rawPower,-max_speed,max_speed);
        if(state == STATE.DRIVE) {
            if(Math.abs(rawPower) < 0.01) {
                finalPower = 0;
            }
        }
        return finalPower;
    }
    public void normalizeArray(double[] arr) {
        double largest = 1;
        for (int i = 0; i < arr.length; i++) {
            largest = Math.max(largest, Math.abs(arr[i]));
        }
        for (int i = 0; i < arr.length; i++) {
            arr[i] /= largest;
        }
    }
    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setPower(clamp(lf,-1,1));
        leftBack.setPower(clamp(lr,-1,1));
        rightBack.setPower(clamp(rr,-1,1));
        rightFront.setPower(clamp(rf,-1,1));
    }
    public boolean isBusy() {
        return state != STATE.IDLE;
    }
    void pidDrive() {
        state = STATE.GOING_TO_POINT;
        xPID.updatePID(xkP,0,xD);
        yPID.updatePID(ykP,0,ykD);
        hPID.updatePID(hkP,0,hkD);

        double xPower = xPID.update(xError,-1,1);
        double yPower = yPID.update(yError,-1,1);
        double hPower = hPID.update(hError,-1,1);

        xPower = Utils.minMaxClip(xPower,-max_speed,max_speed);
        yPower = Utils.minMaxClip(yPower,-max_speed,max_speed);
        hPower = Utils.minMaxClip(hPower,-max_speed,max_speed);

        //gliding mechanism

        if(speed.angVel < 2 && hError < 2 && use_gliding && stop) {
            double current_speed_x = speed.linearVel.x;
            double current_speed_y = speed.linearVel.y; // inch /

            double stopping_distance_x = Math.pow(current_speed_x,2)/(2.0*decelX);
            double stopping_distance_y = Math.pow(current_speed_y,2)/(2.0*deccelY);

            double distance_x = Math.abs(xError);
            double distance_y = Math.abs(yError);

            if(distance_x < stopping_distance_x) {
                xPower = Utils.minMaxClip(xPower,-max_speed*distance_x/stopping_distance_x,max_speed*distance_x/stopping_distance_x);
            }
            if(distance_y < stopping_distance_y) {
                yPower = Utils.minMaxClip(yPower,-max_speed*distance_y/stopping_distance_y,max_speed*distance_y/stopping_distance_y);
            }
        }

        powerVector = new PoseVelocity2d(new Vector2d(xPower,yPower),hPower);

    }

    public void goToPoint(Pose2d targetPoint, boolean finalAdjustment, boolean stop, double maxPower) {
        this.fine_stop = finalAdjustment;
        this.stop = stop;
        this.max_speed = Math.abs(maxPower);
        this.target = targetPoint;

        xPID.reset();
        yPID.reset();
        hPID.reset();
        state = STATE.GOING_TO_POINT;
    }

}
