package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Robot.initMotors;
import static org.firstinspires.ftc.teamcode.Robot.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.random.StableRandomGenerator;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "RightFast", preselectTeleOp = "Teleop Reg.")
public class RightFast extends LinearOpMode {
    final int START_X = 36;
    final int START_Y = -60;

    OpenCvCamera webcam;


    @Override
    public void runOpMode() throws InterruptedException {
        ////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();
        initIMU(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);


        initMotors(this);
        SampleMecanumDrive2 drive = new SampleMecanumDrive2(hardwareMap);

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(90)); //init starting position
        drive.setPoseEstimate(startPose);


        ColorRecognition pipeline = new ColorRecognition(telemetry);
        webcam.setPipeline(pipeline);

        /// CAMERA STUFF
        int parking = 1; // read by camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        redLED.setState(false);
        greenLED.setState(true);

////////Program start////////////////////////////////////////////////////////////////////////
        waitForStart();
        int parkSpot = pipeline.getCount();
        telemetry.update();
        telemetry.addData("spot", parkSpot);
        telemetry.update();
        ////Move on start/init
        claw.setPosition(clawGrab);
        sleep(900);
        linearSlide.setTargetPosition(slideBottom - 80);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.5);
        ////

//        Trajectory inchForward = drive.trajectoryBuilder(startPose) //moves bot forward from start and turns
//                .back(20)
//                .build();

        double parkingLocation;
        double backupN = 10;
        switch (parkSpot) {
            case (1):
                parkingLocation = -20;
                backupN = 96;
                break;
            case (2):
                parkingLocation = 20;
                backupN = 48;
                break;
            case (3):
                parkingLocation = 86;
                backupN = 12;
                break;
            default:
                parkingLocation = 77;
                break;


        }
        //(-36, 60)
//        Trajectory inchForward = drive.trajectoryBuilder(startPose)
//                .back(6)
//                .build();
//        //(-36, 54)
//        Trajectory toLeft = drive.trajectoryBuilder(inchForward.end())
//                .strafeRight(42)
//                .build();
//        //(8, 54)
//        Trajectory toHighJunc = drive.trajectoryBuilder(toLeft.end())
//                .back(29)
//                .build();
//        //(8, 25)
        double turnAngle = 46;
        //start pose
        Trajectory toJuncCurv = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-2, -48), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-3, -11, Math.toRadians(119)), Math.toRadians(90))
                .build();
        Trajectory littleBackup = drive.trajectoryBuilder(toJuncCurv.end())
                .back(5)
                .build();
        Trajectory backup = drive.trajectoryBuilder(toJuncCurv.end())
                .back(17)
                .build();
        Trajectory toConeStack1 = drive.trajectoryBuilder(backup.end().plus(new Pose2d(-6, 6)))
                .lineToLinearHeading(new Pose2d(18, 14, Math.toRadians(343)))
                .build();
        Trajectory toConeStack2 = drive.trajectoryBuilder(toConeStack1.end().plus(new Pose2d(0, 0, Math.toRadians(25))))
                .forward(60)
                .build();

        Trajectory backup2 = drive.trajectoryBuilder(new Pose2d(30, -6, 100))
                .back(backupN)
                .build();
        Trajectory backup3 = drive.trajectoryBuilder(new Pose2d(56, -12, 0))
                .back(7)
                .build();
        Trajectory toJunc2 = drive.trajectoryBuilder(backup3.end())
                .lineToLinearHeading(new Pose2d(35, -26, 42))
                .build();
        Trajectory park = drive.trajectoryBuilder(new Pose2d(48, -24, Math.toRadians(225)))
                        .forward(66)
                                .build();
        Trajectory park2 = drive.trajectoryBuilder(new Pose2d(48, -24, Math.toRadians(225)))
                .lineTo(new Vector2d(0, -24))
                .build();
        Trajectory park3 = drive.trajectoryBuilder(new Pose2d(48, -24, Math.toRadians(225)))
                .lineTo(new Vector2d(55, -30))
                        .build();


        sleep(200);


        drive.followTrajectory(toJuncCurv);


        linearSlide.setTargetPosition(slideHigh + 50);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideFast);
        sleep(1200);

        ElapsedTime timer = new ElapsedTime();
        Boolean sensorMissed = false;


        while (!seeJunction()) {
            if (seeJunction()) break;
            if (timer.milliseconds() / 1000 > 3) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            SetPower(-.125, .125, -.125, .125);
        }
        SetPower(0, 0, 0, 0);
        timer.reset();
        if(sensorMissed){
            while(distanceF.getDistance(DistanceUnit.MM) > 320){
                if(timer.milliseconds() / 1000 > 3) break;
                SetPower(.125, -.125, .125, -.125);
            }
            SetPower(0, 0, 0, 0);

        }


        timer.reset();

        while (!detectJunction()) {
            if (detectJunction()) break;
            if (timer.milliseconds() / 1000 > 5) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            SetPower(.095, .095, .095, .095);
        }
        SetPower(0, 0, 0, 0);
        if(sensorMissed){
            drive.followTrajectory(backup);
        }
        claw.setPosition(clawOpen);

        sleep(100);
        drive.followTrajectory(backup);
        linearSlide.setTargetPosition(-675);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideSlow);
        sleep(100);

        drive.followTrajectory(toConeStack1);
//        drive.turn(Math.toRadians((0-imu.getAngularOrientation().firstAngle) * 10));
        // Connors make-shift super accurate turn
        while(Math.toDegrees(imu.getAngularOrientation().firstAngle) < -94){
            telemetry.addData("IMU", Math.toDegrees(imu.getAngularOrientation().firstAngle));
            telemetry.update();
            SetPower(-.12, .12, -.12, .12);
        }
        SetPower(0, 0, 0, 0);
        sleep(120);
        if(Math.toDegrees(imu.getAngularOrientation().firstAngle) > -88){
            while(Math.toDegrees(imu.getAngularOrientation().firstAngle) > -88)
                SetPower(.12, -.12, .12, -.12);
        }
        SetPower(0, 0,0, 0);

        drive.followTrajectory(toConeStack2);


//
// GRAB SECOND CONE

        sleep(350);
        timer.reset();
        while (!seeConeStack()) {
            if (seeConeStack()) break;
            if (timer.milliseconds() / 1000 > 5) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            SetPower(.2, -.2, -.2, .2);
        }
        SetPower(0, 0, 0, 0);


//        timer.reset();
//        {
//
//            SetPower(.2, -.2, -.2, .2);
//
//        }




        timer.reset();

        while (distanceF.getDistance(DistanceUnit.MM) > 70) {
            if (timer.milliseconds() / 1000 > 2.5) {
                break;
            }
            SetPower(.2, .2, .2, .2);
        }
        SetPower(0, 0, 0, 0);
        sleep(300);
        //-595 encoder value
        claw.setPosition(clawGrab);
        sleep(250);

        linearSlide.setTargetPosition(slideLow - 400);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideSlow);
        sleep(200);



        drive.followTrajectory(backup3);
        drive.followTrajectory(toJunc2);


//      SCORE SECOND CONE
        sensorMissed = false;
        timer.reset();
        while (!seeJunction2()) {
            if (seeJunction2()) break;
            if (timer.milliseconds() / 1000 > 1.7) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            SetPower(.15, -.15, .15, -.15);
        }
        sleep(350);
        SetPower(0, 0, 0, 0);


        if(sensorMissed) {

            while(Math.abs(Math.toDegrees(imu.getAngularOrientation().firstAngle) - 90) > 4){
                SetPower(-.115, .115, -.115, .115);
            }
            SetPower(0, 0, 0, 0);


        }
        timer.reset();
        while (!detectJunction2()) {
            if (timer.milliseconds() / 1000 > 5) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            SetPower(.1, .1, .1, .1);
        }
        SetPower(0, 0, 0, 0);
        claw.setPosition(clawOpen);

        sleep(100);
        drive.followTrajectory(backup);





        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideFast);
        sleep(100);

        if(parkSpot == 3){
            drive.followTrajectory(park3);
        }else if(parkSpot == 2){
            drive.followTrajectory(park2);
        } else {
            while(Math.toDegrees(imu.getAngularOrientation().firstAngle) > 100){
                telemetry.addData("IMU", Math.toDegrees(imu.getAngularOrientation().firstAngle));
                telemetry.update();
                SetPower(.4, -.4, .4, -.4);
            }
            SetPower(0, 0, 0, 0);
            drive.followTrajectory(park);
        }



        if (isStopRequested()) return;
        sleep(5000);


    }




}