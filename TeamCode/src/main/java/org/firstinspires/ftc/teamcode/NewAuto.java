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


@Autonomous(name = "RightMiddle", preselectTeleOp = "Teleop Reg.")
public class NewAuto extends LinearOpMode {
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

        final double flipperD = .955;



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
        sleep(200);
        linearSlide.setTargetPosition(slideLow-100);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.5);
        ////


        double parkingLocation;
        double backupN = 10;
        switch (parkSpot) {
            case (1):
                parkingLocation = -20;
                backupN = 1;
                break;
            case (2):
                parkingLocation = 20;
                backupN = 20;
                break;
            case (3):
                parkingLocation = 69;
                backupN = 55;
                break;
            default:
                parkingLocation = 77;
                backupN = 1;
                break;


        }
        //PLOW SIGNAL CONE TO GET TO THE JUNCTION
        Trajectory junc1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(startPose.getX() - 8, 12, Math.toRadians(96))) //72
                .build();
        Trajectory junc1Backup = drive.trajectoryBuilder(junc1.end())
                .lineToLinearHeading(new Pose2d(junc1.end().getX() - 15, -15, Math.toRadians(94)))
                .build();
        Trajectory strafe1 = drive.trajectoryBuilder(junc1Backup.end())
                .strafeRight(2)
                .build(); //Not really used
        Trajectory failsafeBack = drive.trajectoryBuilder(strafe1.end())
                .back(6)
                .build();
        Trajectory normalBack = drive.trajectoryBuilder(strafe1.end())
                .back(3)
                .build();
        Trajectory backup = drive.trajectoryBuilder(strafe1.end())
                .back(15.5)
                .build();

        Trajectory toConeStack1 = drive.trajectoryBuilder(new Pose2d(30,-7, Math.toRadians(135)))
                .lineToLinearHeading(new Pose2d(19, 27.5, Math.toRadians(40)))
                .build();

        Pose2d toConeStack1End = new Pose2d(30, -7, Math.toRadians(0));

        Trajectory toConeStack2 = drive.trajectoryBuilder(toConeStack1End)
                .forward(30)
                .build();

        Trajectory toJunction2 = drive.trajectoryBuilder(toConeStack2.end())
                .lineToLinearHeading(new Pose2d(8, -20, Math.toRadians(348)))
                .build();


        Trajectory park1 = drive.trajectoryBuilder(toConeStack1.end())
                .forward(backupN)
                .build();





        while(linearSlide.getCurrentPosition() > slideLow + 250) {}
        flipper.setPosition(flipperD);
        sleep(300);
        drive.followTrajectory(junc1);
        flipper.setPosition(flipperUp);

        drive.followTrajectory(junc1Backup);


        // TURN TO FIND THE JUNCTION


        linearSlide.setTargetPosition(slideMedium - 220);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.75);
        sleep(50);


        drive.turn(Math.toRadians(47));

        ElapsedTime timer = new ElapsedTime();
        Boolean sensorMissed = false;
        setbaselineDistance(distanceF.getDistance(DistanceUnit.MM));
        while (distanceF.getDistance(DistanceUnit.MM) > 0.90 * getBaselineDistance()) {
            if (distanceF.getDistance(DistanceUnit.MM) < 0.90 * getBaselineDistance()) break;
            if (timer.milliseconds() / 1000 > 3) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            drive.SetVelocity(-.1025, .1025, -.1025, .1025);
        }
        sleep(200);
        drive.SetVelocity(0, 0, 0, 0);

        drive.followTrajectoryAsync(strafe1);

        timer.reset();

        setBaselineDistanceH(distanceH.getDistance(DistanceUnit.MM));
        while (distanceH.getDistance(DistanceUnit.MM) > 0.85 * getBaselineDistanceH()) {
            if (distanceH.getDistance(DistanceUnit.MM) < 0.85 * getBaselineDistanceH()) break;
            if (timer.milliseconds() / 1000 > 4) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                drive.followTrajectory(failsafeBack);
                break;
            }
            drive.SetVelocity(.1125, .1125, .1125, .1125);
        }
        drive.SetVelocity(0, 0, 0, 0);
        drive.followTrajectory(normalBack);


        claw.setPosition(clawOpen);

        drive.followTrajectory(backup);

        // get ready to grap second cone

        linearSlide.setTargetPosition(-675);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideSlow);
        sleep(200);

        drive.followTrajectory(toConeStack1);
        while(Math.toDegrees(imu.getAngularOrientation().firstAngle) < -94){
            telemetry.addData("IMU", Math.toDegrees(imu.getAngularOrientation().firstAngle));
            telemetry.update();
            drive.SetVelocity(-.15, .15, -.15, .15);
        }
        drive.SetVelocity(0, 0, 0, 0);
        sleep(120);
        if(Math.toDegrees(imu.getAngularOrientation().firstAngle) > -88){
            while(Math.toDegrees(imu.getAngularOrientation().firstAngle) > -88)
                drive.SetVelocity(.15, -.15, .15, -.15);
        }
        drive.SetVelocity(0, 0,0, 0);



        linearSlide.setTargetPosition(-675);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideSlow);
        sleep(100);


        //GO FORWARD AND GRAB CONE

        drive.followTrajectory(toConeStack2);

        timer.reset();
        while (!seeConeStack()) {
            if (seeConeStack()) break;
            if (timer.milliseconds() / 1000 > 5) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            drive.SetVelocity(-.2, .2, .2, -.2);
        }
        sleep(400);
        drive.SetVelocity(0, 0, 0, 0);

        timer.reset();
        while (distanceH.getDistance(DistanceUnit.MM) > 180) {
            if (timer.milliseconds() / 1000 > 3.25) {
                break;
            }
            drive.SetVelocity(.2, .2, .2, .2);
        }
        drive.SetVelocity(0, 0, 0, 0);

        claw.setPosition(clawGrab);
        sleep(400);

        linearSlide.setTargetPosition(slideLow - 400);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideSlow);
        sleep(100);

        drive.followTrajectory(toJunction2);



        // SCORE SECOND CONE

        linearSlide.setTargetPosition(slideMedium - 220);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.75);

        drive.turn(Math.toRadians(-45));

        timer.reset();
        while (!seeJunction2()) {
            if (seeJunction2()) break;
            if (timer.milliseconds() / 1000 > 3) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            drive.SetVelocity(.11, -.11, .11, -.11);
        }
        sleep(400);
        drive.SetVelocity(0, 0, 0, 0);

        while (!detectJunction()) {
            if (detectJunction()) break;
            if (timer.milliseconds() / 1000 > 6) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            drive.SetVelocity(.1225, .1225, .1225, .1225);
        }
        drive.SetVelocity(0, 0, 0, 0);

        claw.setPosition(clawOpen);
        sleep(200);

        drive.followTrajectory(backup);

        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideFast);


        while(Math.toDegrees(imu.getAngularOrientation().firstAngle) < -94){
            telemetry.addData("IMU", Math.toDegrees(imu.getAngularOrientation().firstAngle));
            telemetry.update();
            drive.SetVelocity(-.2, .2, -.2, .2);
        }
        drive.SetVelocity(0, 0, 0, 0);
        sleep(120);
        if(Math.toDegrees(imu.getAngularOrientation().firstAngle) > -88){
            while(Math.toDegrees(imu.getAngularOrientation().firstAngle) > -88)
                drive.SetVelocity(.15, -.15, .15, -.15);
        }
        drive.SetVelocity(0, 0,0, 0);










        drive.followTrajectory(park1);




        if (isStopRequested()) return;
        sleep(5000);


    }




}