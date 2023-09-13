package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Robot.SetPower;
import static org.firstinspires.ftc.teamcode.Robot.claw;
import static org.firstinspires.ftc.teamcode.Robot.clawGrab;
import static org.firstinspires.ftc.teamcode.Robot.clawOpen;
import static org.firstinspires.ftc.teamcode.Robot.detectJunction;
import static org.firstinspires.ftc.teamcode.Robot.distanceF;
import static org.firstinspires.ftc.teamcode.Robot.distanceH;
import static org.firstinspires.ftc.teamcode.Robot.flipper;
import static org.firstinspires.ftc.teamcode.Robot.flipperUp;
import static org.firstinspires.ftc.teamcode.Robot.getBaselineDistanceH;
import static org.firstinspires.ftc.teamcode.Robot.greenLED;
import static org.firstinspires.ftc.teamcode.Robot.imu;
import static org.firstinspires.ftc.teamcode.Robot.initIMU;
import static org.firstinspires.ftc.teamcode.Robot.initMotors;
import static org.firstinspires.ftc.teamcode.Robot.limit;
import static org.firstinspires.ftc.teamcode.Robot.linearSlide;
import static org.firstinspires.ftc.teamcode.Robot.redLED;
import static org.firstinspires.ftc.teamcode.Robot.seeJunction;
import static org.firstinspires.ftc.teamcode.Robot.setBaselineDistanceH;
import static org.firstinspires.ftc.teamcode.Robot.setbaselineDistance;
import static org.firstinspires.ftc.teamcode.Robot.slideLow;
import static org.firstinspires.ftc.teamcode.Robot.slideMedium;
import static org.firstinspires.ftc.teamcode.Robot.slideSlow;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "LeftMiddle", preselectTeleOp = "Teleop Reg.")
public class NewAutoLeft extends LinearOpMode {
    final int START_X = -36;
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
        sleep(900);
        linearSlide.setTargetPosition(slideLow-100);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.5);
        ////


        double parkingLocation;
        double backupN = -6;
        switch (parkSpot) {
            case (1):
                parkingLocation = -20;
                backupN = -32;
                break;
            case (2):
                parkingLocation = 20;
                backupN = 3;
                break;
            case (3):
                parkingLocation = 86;
                backupN = 32;
                break;
            default:
                parkingLocation = 77;
                backupN = -3;
                break;


        }
        //PLOW SIGNAL CONE TO GET TO THE JUNCTION
        Trajectory junc1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(startPose.getX() - 5, 12, Math.toRadians(92)))
                .build();
        Trajectory junc1Backup = drive.trajectoryBuilder(junc1.end())
                .lineToLinearHeading(new Pose2d(startPose.getX() - 13, -17, Math.toRadians(95)))
                .build();

        Trajectory strafe1 = drive.trajectoryBuilder(junc1Backup.end())
                .strafeRight(2)
                .build(); //Not really used

        Trajectory safetyBackup = drive.trajectoryBuilder(new Pose2d())
                .back(6)
                .build();
        Trajectory backup = drive.trajectoryBuilder(strafe1.end())
                .back(15)
                .build();
        Trajectory park1 = drive.trajectoryBuilder(backup.end())
                .forward(backupN)
                .build();

        Trajectory normalBackup = drive.trajectoryBuilder(new Pose2d())
                .back(3)
                .build();






        while(linearSlide.getCurrentPosition() > slideLow + 250) {}
        flipper.setPosition(flipperD);
        sleep(500);
        drive.followTrajectory(junc1);
        flipper.setPosition(flipperUp);

        drive.followTrajectory(junc1Backup);


        // TURN TO FIND THE JUNCTION


        linearSlide.setTargetPosition(slideMedium - 220);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.75);
        sleep(450);


        drive.turn(Math.toRadians(-50));

        ElapsedTime timer = new ElapsedTime();
        Boolean sensorMissed = false;
        while (!seeJunction()) {
            if (seeJunction()) break;
            if (timer.milliseconds() / 1000 > 3) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            drive.SetVelocity(.1125, -.1125, .1125, -.1125);
        }
        sleep(400);
        drive.SetVelocity(0, 0, 0, 0);

//        drive.followTrajectory(strafe1);

        timer.reset();
        sensorMissed = false;
        setBaselineDistanceH(distanceH.getDistance(DistanceUnit.MM));
        while (distanceH.getDistance(DistanceUnit.MM) > 0.8 * getBaselineDistanceH()) {
            if (distanceH.getDistance(DistanceUnit.MM) < 0.8 * getBaselineDistanceH()) break;
            if (timer.milliseconds() / 1000 > 6) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(", "");
                break;
            }
            drive.SetVelocity(.115, .115, .115, .115);
        }
        drive.SetVelocity(0, 0, 0, 0);
        sleep(200);
        if(sensorMissed) drive.followTrajectory(safetyBackup);
        else drive.followTrajectory(normalBackup);
        claw.setPosition(clawOpen);
        drive.followTrajectory(backup);

        // get ready to grap second cone

        linearSlide.setTargetPosition(-675);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideSlow);
        sleep(200);


        while(Math.toDegrees(imu.getAngularOrientation().firstAngle) < -86){
            telemetry.addData("IMU", Math.toDegrees(imu.getAngularOrientation().firstAngle));
            telemetry.update();
            drive.SetVelocity(-.15, .15, -.15, .15);
        }
        drive.SetVelocity(0, 0, 0, 0);
        sleep(120);
        if(Math.toDegrees(imu.getAngularOrientation().firstAngle) > -92){
            while(Math.toDegrees(imu.getAngularOrientation().firstAngle) > -92)
                drive.SetVelocity(.15, -.15, .15, -.15);
        }
        drive.SetVelocity(0, 0,0, 0);


        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideSlow);

        sleep(225);
        drive.followTrajectory(park1);




        if (isStopRequested()) return;
        sleep(5000);


    }




}