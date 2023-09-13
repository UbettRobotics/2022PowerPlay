package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive2;


@TeleOp(name = "Outreach")
public class linearslideTest01 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);
        SampleMecanumDrive2 drive = new SampleMecanumDrive2(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90)); //init starting position
        drive.setPoseEstimate(startPose);

        DcMotor linearSlide;
        linearSlide = hardwareMap.get(DcMotor.class, "linearslide");
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int currentLevel = 0;
        final int MAXLEVEL = -4444;

        waitForStart();

        Trajectory forward = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(40)
                .build();



        while (opModeIsActive()) {
            boolean LBumper1 = gamepad1.left_bumper;
            boolean RBumper1 = gamepad1.right_bumper;

            double LStickY = gamepad1.left_stick_y;
            double LStickX = gamepad1.left_stick_x;
            double RStickY = -gamepad1.right_stick_y;
            double RStickX = -gamepad1.right_stick_x;

            double LTrigger1 = gamepad1.left_trigger; //
            double RTrigger1 = gamepad1.right_trigger; //

            boolean a1 = gamepad1.a;
            boolean b1 = gamepad1.b;
            boolean x1 = gamepad1.x;
            boolean y1 = gamepad1.y;

            boolean a2 = gamepad2.a;
            boolean b2 = gamepad2.b;
            boolean x2 = gamepad2.x;
            boolean y2 = gamepad2.y;

            double LTrigger2 = gamepad2.left_trigger;
            double RTrigger2 = gamepad2.right_trigger;
            boolean LBumper2 = gamepad2.left_bumper;
            boolean RBumper2 = gamepad2.right_bumper;

            double RStickY2 = -gamepad2.right_stick_y;
            double RStickX2 = gamepad2.right_stick_x;
            double LStickY2 = -gamepad2.left_stick_y;
            double LStickX2 = gamepad2.left_stick_x;

            boolean dpadUp1 = gamepad1.dpad_up;
            boolean dpadDown1 = gamepad1.dpad_down;
            boolean dpadRight1 = gamepad1.dpad_right;
            boolean dpadLeft1 = gamepad1.dpad_left;

            boolean dpadUP2 = gamepad2.dpad_up;
            boolean dpadDOWN2 = gamepad2.dpad_down;
            boolean dpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;

            boolean RStickIn1 = gamepad1.right_stick_button;
            boolean LStickIn1 = gamepad1.left_stick_button;

            boolean RStickIn2 = gamepad2.right_stick_button;
            boolean LStickIn2 = gamepad2.left_stick_button;

            currentLevel = linearSlide.getCurrentPosition();

//            if (a1 && currentLevel <= 1) { // down
//                linearSlide.setPower(.2);
//            } else if (b1 && currentLevel >= MAXLEVEL) { // up
//                linearSlide.setPower(-.2);
//            } else {
//                linearSlide.setPower(0);
//            }
//            if(LBumper1) {
//                linearSlide.setPower(-0.15);
//            } else if (RBumper1){
//                linearSlide.setPower(.15);
//            } else {
//                linearSlide.setPower(0);
//            }
////
////

//            if(a1){
//               SetPower(1, 0, 0, 0);
//            }
//            else if(b1){
//                SetPower(0, 1, 0, 0);
//            }
//            else if(y1){
//                SetPower(0, 0, 1, 0);
//            }
//            else if(x1){
//               SetPower(0, 0, 0, 1);
//            } else {
//                SetPower(0, 0, 0, 0);
//            }


//            if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {
//
//                /*
//                if (Math.abs(LStickX) < .05 && Math.abs(RStickX) < .05) {
//                    SetPower(LStickY, LStickY, LStickY, LStickY);
//                } else if (Math.abs(LStickY) < .05 && Math.abs(RStickX) < .05) {
//                    SetPower(LStickX, -LStickX, -LStickX, LStickX);//+--+
//                } else {
//                 */
//
//
//                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//                double rightX = -gamepad1.right_stick_x;
//
//                double v1 = r * Math.cos(robotAngle) + rightX; //lf
//                double v2 = r * Math.sin(robotAngle) - rightX; //rf
//                double v3 = r * Math.sin(robotAngle) + rightX; //lb
//                double v4 = r * Math.cos(robotAngle) - rightX; //rb
//
//                SetPower(v1, v2, v3, v4);
//            }
//            else SetPower(0,0,0,0);

//            if(x1 && !detectJunction()) {
//                SetPower(-.2,-.2, -.2, -.2);
//            } else {
//                SetPower(0, 0, 0, 0);
//            }
            drive.followTrajectory(forward);

            //telemetry/////////////////////////////////////////////////////////////////////////////
            telemetry.addData("detect", detectJunction());

            telemetry.update();

        }


    }
}

