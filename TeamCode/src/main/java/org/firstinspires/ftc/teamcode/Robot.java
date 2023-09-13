package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import  com.qualcomm.robotcore.hardware.TouchSensor;



public class Robot {
    public Robot(){}

    static DcMotor rightfront;
    static DcMotor leftfront;
    static DcMotor leftback;
    static DcMotor rightback;

    static DcMotor linearSlide;
    static Servo claw;
    static Servo flipper;

    static TouchSensor limitSwitch;
    static DistanceSensor distanceF;
    static DistanceSensor distanceH;
    static TouchSensor limit;
//    static VoltageSensor voltage;
    static BNO055IMU imu;
    static BNO055IMU imu2;
    static GyroSensor gyro;
    static DigitalChannel redLED;
    static DigitalChannel greenLED;
    static Orientation lastAngles = new Orientation();
    static double globalAngle, correction;
    static double baselineDistance;
    static double baselineDistanceH;


    // JUNCTIONS
    static final int slideHigh = -4058;
    static final int slideLow = -1432;
    static final int slideGrab = -0;
    static final int slideBottom = 8;
    static final int slideMedium = -2713;

    // CONE STACK HEIGHTS

    static final int[] coneHeights = {0, -180, -315, -450, -585};


    static int currentCone = 5;


    static final int beaconHeight = -125;

    static final double slideFast = .98;
    static final double slideSlow = .95;

    static final double clawOpen = .569; //gold .545; // blue grabber .5125;
    static final double clawGrab = .6;//gold .6;// blue grabber .54;

    static final double flipperDown = 0.90;
    static final double flipperUp = 0.46;




    public static void initMotors(OpMode opMode){
        rightfront = opMode.hardwareMap.get(DcMotor.class, "rightfront");
        leftfront = opMode.hardwareMap.get(DcMotor.class, "leftfront");
        leftback = opMode.hardwareMap.get(DcMotor.class, "leftback");
        rightback = opMode.hardwareMap.get(DcMotor.class, "rightback");

        linearSlide = opMode.hardwareMap.get(DcMotor.class, "linearslide");
        claw = opMode.hardwareMap.get(Servo.class, "claw");
        flipper = opMode.hardwareMap.get(Servo.class, "flipper");

        limitSwitch = opMode.hardwareMap.get(TouchSensor.class, "limitSwitch");
        distanceF = opMode.hardwareMap.get(DistanceSensor.class, "forwardDistance");
        distanceH = opMode.hardwareMap.get(DistanceSensor.class, "distanceHoriz");
//        voltage = opMode.hardwareMap.voltageSensor.get("voltage");
        limit = opMode.hardwareMap.get(TouchSensor.class, "limit");

        redLED = opMode.hardwareMap.get(DigitalChannel.class, "red");
        greenLED = opMode.hardwareMap.get(DigitalChannel.class, "green");
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);




        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        resetMotors();
    }
    public static void resetMotors() {
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public static void SetPower(double LFPower, double RFPower, double LBPower, double RBPower) {
        //leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftfront.setPower(LFPower);
        leftback.setPower(LBPower);
        rightfront.setPower(RFPower);
        rightback.setPower(RBPower);


    }

    public static void initIMU(OpMode opMode){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        //This line is only necessary if the the Control Hub is mounted vertically (as done this year)
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = opMode.hardwareMap.get(BNO055IMU.class, "imu2");

        imu.initialize(parameters);
        imu2.initialize(parameters);

        opMode.telemetry.addData("Mode: ", "imu calibrating");
        opMode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated() || !imu2.isGyroCalibrated()) {
        }
        opMode.telemetry.addData("imu calib status: ", imu.getCalibrationStatus().toString());
        opMode.telemetry.update();
        resetAngle();

    }
    private static void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation lastAngles2 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public static float getAvgHeading() {
        double angle1 = Math.toRadians(imu.getAngularOrientation().firstAngle);
        double angle2 = Math.toRadians(imu2.getAngularOrientation().firstAngle);
        //angle2 = (angle2 + 180) % 360;
        double a = Math.atan2(Math.sin(angle1) + Math.sin(angle2)  ,  Math.cos(angle1) + Math.cos(angle2));
        //Degree return: return Math.round(Math.toDegrees(a)); //round to long/int, cast to float as returned by imu;
        return (float)a;
    }

    // detect = horizontal detectoin / somehting being in the robot
    // see = forwrad detection / something in front of robot
    public static boolean detectJunction() { // horizontal distance sensor within robot
        return (distanceH.getDistance(DistanceUnit.MM) < 200 && distanceH.getDistance(DistanceUnit.INCH) > 0);
    }
    public static boolean seeJunction(){ // distance sensor facing outwards
        return (distanceF.getDistance(DistanceUnit.MM) < 294 && distanceF.getDistance(DistanceUnit.MM) > 0); //280
    }
    public static boolean seeConeStack(){
        return (distanceF.getDistance(DistanceUnit.MM) < 300 && distanceF.getDistance(DistanceUnit.MM) > 0);
        //<24
    }
    public static boolean detectConestack(){
        return (distanceH.getDistance(DistanceUnit.MM) < 4 && distanceH.getDistance(DistanceUnit.MM) > 0);
    }
    public static boolean seeJunction2(){
        return (distanceF.getDistance(DistanceUnit.MM) < 260);
    }
    public static boolean detectJunction2(){
        return (distanceH.getDistance(DistanceUnit.MM) < 142);
    }

    public static void setbaselineDistance(double dist) {
        baselineDistance = dist;
    }
    public static void setBaselineDistanceH(double dist) {
        baselineDistanceH = dist;
    }
    public static double getBaselineDistance() {
        return baselineDistance;
    }
    public static double getBaselineDistanceH(){
        return baselineDistanceH;
    }

}
