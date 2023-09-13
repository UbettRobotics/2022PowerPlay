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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "RightField", preselectTeleOp = "Teleop Reg.")
public class Right extends LinearOpMode{
    final int START_X = -36;
    final int START_Y = 60;

    OpenCvCamera webcam;



    @Override
    public void runOpMode() throws InterruptedException {
        ////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        initMotors(this);
        initIMU(this);

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(270)); //init starting position
        drive.setPoseEstimate(startPose);


        ColorRecognition pipeline = new ColorRecognition(telemetry);
        webcam.setPipeline(pipeline);

        /// CAMERA STUFF
        int parking = 1; // read by camera
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });




////////Program start////////////////////////////////////////////////////////////////////////

        waitForStart();
        int parkSpot = pipeline.getCount();
        telemetry.update();
        telemetry.addData("spot", parkSpot);
        telemetry.update();
        ////Move on start/init
        claw.setPosition(clawGrab);
        sleep(1750);
        linearSlide.setTargetPosition(slideBottom);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.5);
        ////


//        Trajectory inchForward = drive.trajectoryBuilder(startPose) //moves bot forward from start and turns
//                .back(20)
//                .build();

        double strafe;
        switch (parkSpot){
            case (1):
                strafe = 0.1;
                break;
            case(2):
                strafe = -51;
                break;
            case(3):
                strafe = -105;
                break;
            default:
                strafe = 0.1;


        }
        //(-36, 60)
        Trajectory inchForward = drive.trajectoryBuilder(startPose)
                .back(6)
                .build();
        //(-36, 54)
        Trajectory toLeft = drive.trajectoryBuilder(inchForward.end())
                .strafeRight(42)
                .build();
        //(8, 54)
        Trajectory toHighJunc = drive.trajectoryBuilder(toLeft.end())
                .back(34)
                .build();
        //(8, 25)
        double turnAngle = 50;
        Trajectory toJuncCurv = drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(8, 25), Math.toRadians(270))
                .build();
        Trajectory getCloser = drive.trajectoryBuilder(toHighJunc.end().plus(new Pose2d(0, 0, Math.toRadians(-turnAngle))))
                .back(14)
                .build();
        Trajectory backup = drive.trajectoryBuilder(getCloser.end())
                .forward(5)
                .build();
        Trajectory parkLeft = drive.trajectoryBuilder(getCloser.end())
                        .forward(19)
                                .build();
        Trajectory park = drive.trajectoryBuilder(parkLeft.end())
                .strafeRight(strafe)
                .build();





        drive.followTrajectory(inchForward);
        drive.followTrajectory(toLeft);
        drive.followTrajectory(toHighJunc);
        //drive.turn(turnAngle);

        ElapsedTime timer = new ElapsedTime();
        Boolean sensorMissed = false;

        linearSlide.setTargetPosition(slideHigh);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slideFast);
        sleep(500);

//        while (!seeJunction()) {
//            if (seeJunction()) break;
//            if (timer.milliseconds() / 1000 > 8 || imu.getAngularOrientation().firstAngle > 70) {
//                sensorMissed = true;
//                telemetry.addData("I MIssed :(", "");
//                break;
//            }
//            SetPower(.15, -.15, .15, -.15);
//        }
        sleep(75);
        SetPower(0, 0, 0, 0);



        timer = new ElapsedTime();
        sensorMissed = false;
        timer.reset();

        while (!detectJunction()) {
            if(detectJunction()) break;
            if(timer.milliseconds()/1000 > 5) {
                sensorMissed = true;
                telemetry.addData("I MIssed :(","");
                break;
            }
            SetPower(-.125,-.125, -.125, -.125);
        }
        SetPower(0, 0, 0, 0);
        sleep(400);
        if (sensorMissed) {
            drive.followTrajectory(backup);
        }
        claw.setPosition(clawOpen);
        sleep(1000);

        drive.followTrajectory(parkLeft);
        drive.turn(Math.toRadians(turnAngle + 14));
        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(.6);

        drive.followTrajectory(park);
        telemetry.addData("park", parkSpot);
        telemetry.update();



        if (isStopRequested()) return;
        sleep(5000);


    }
}