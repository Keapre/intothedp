//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.Utils.Caching.OPColorSensor;
//import org.firstinspires.ftc.teamcode.Utils.Dashboard.TelemetryUtil;
//
//public class Sensors {
//    HardwareMap hardwareMap;
//    private LynxModule controlHub, expansionHub;
//
//    OPColorSensor clawSensor;
//    Limelight3A limelight;
//
//    public enum LimeLightMode {
//        ATAG(1),
//        SAMPLE_DETECTION(0);
//
//        public int pipeline;
//        LimeLightMode(int pipeline) {
//            this.pipeline = pipeline;
//        }
//    }
//
//    public boolean useLimeLight = true;
//    public boolean useClawSensor = true;
//
//    private double voltage;
//
//    private double voltageUpdateTime = 5000;
//    long lastVoltageUpdatedTime = System.currentTimeMillis();
//
//    private double colorUpdate = 100;
//    long lastColorUpdate = System.currentTimeMillis();
//    public boolean huskyJustUpdated = false;
//    private double leftFrontMotorCurrent, leftRearMotorCurrent, rightRearMotorCurrent, rightFrontMotorCurrent;
//
//    Robot robot;
//    public LimeLightMode limeLightMode = LimeLightMode.ATAG;
//
//    public Sensors(Robot robot,HardwareMap hardwareMap) {
//        clawSensor = new OPColorSensor(hardwareMap.get(ColorRangeSensor.class, "clawSensor"));
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        this.robot = robot;
//        this.hardwareMap = hardwareMap;
//        initSensors();
//    }
//    private void initSensors() {
//        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
//        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//
//
//
//        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
//        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//
//        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//
//        limelight.setPollRateHz(100);
//        limelight.start();
//        lastVoltageUpdatedTime = System.currentTimeMillis();
//    }
//
//    public void update() {
//        updateControlHub();
//    }
//
//    public void updateControlHub(){
//        long currTime = System.currentTimeMillis();
//        if(currTime - lastVoltageUpdatedTime > voltageUpdateTime) {
//            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//            lastVoltageUpdatedTime = currTime;
//        }
//        TelemetryUtil.packet.put("Voltage",voltage);
//
//        if(useLimeLight) {
//            //TODO
//        }
//        if(useClawSensor && currTime-lastColorUpdate > colorUpdate) {
//            clawSensor.update();
//            lastColorUpdate = currTime;
//        }
//    }
//
//    public double getVoltage() {
//        return voltage;
//    }
//
//    public double getDistanceSensor() {
//        return clawSensor.getDistance();
//    }
//}
