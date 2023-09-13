package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@TeleOp(name = "Teleop Reg.")
public class TeleopRegular extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors(this);
        initIMU(this);
        ElapsedTime mRunTime = new ElapsedTime(); //sets up a timer in the program4
        boolean start = false;
        double damperN = 0;
        long startTime = 0;
        double clawPos = clawOpen;
        boolean manual = false;
        boolean fixClaw = false;
        int flipper_status = 0; // 0 & 2 are tested values. 1 & 3 are used as temp until stick released
        int square_state = 0; // 0 is not pressed // 1 is pressed // 2 is grab cone
        boolean dist = true;
        int ledMode = 1; /* 1 = green = claw open and not close enough
                            2 = flashing = close enough
                            3 = red = claw closed
        */
        boolean coneFallEdge = false;


        boolean flashLED = false;
        if(!limit.isPressed()){
            fixClaw = true;
        }

        waitForStart();
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

            boolean dpadUp2 = gamepad2.dpad_up;
            boolean dpadDown2 = gamepad2.dpad_down;
            boolean dpadRight2 = gamepad2.dpad_right;
            boolean dpadLeft2 = gamepad2.dpad_left;

            boolean LStickIn = gamepad1.left_stick_button;
            boolean LStickIn2 = gamepad2.left_stick_button;

            boolean RStickIn2 = gamepad2.right_stick_button;

            if (Math.abs(LStickX) > 0 || Math.abs(LStickY) > 0 || Math.abs(RStickX) > 0) {


                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = -gamepad1.right_stick_x / 2;

                double v1 = r * Math.sin(robotAngle) + rightX; //lf // wsa cos
                double v2 = r * Math.cos(robotAngle) - rightX; //rf // was sin
                double v3 = r * Math.cos(robotAngle) + rightX; //lb // was sin but swtiched
                double v4 = r * Math.sin(robotAngle) - rightX; //rb // was cos

                //equation time
                if (!start) {
                    startTime = System.currentTimeMillis();
                    damperN = 0;
                    start = true;
                }
                int K = 5;
                double damper = Math.pow(2, (4 * damperN) - 3) + 0.5;
                if ((System.currentTimeMillis() - startTime) % K == 0) {
                    damperN += 0.01;
                }
                if (Math.abs(RStickX) > 0) {
                    v1 /= damper; // v1 = v1 / damper
                    v2 /= damper;
                    v3 /= damper;
                    v4 /= damper;
                }
                SetPower(v1 * damper, v2 * damper, v3 * damper, v4 * damper); // v1 / damper * damper
            } else if (RTrigger1 > 0.05) {
                SetPower(-RTrigger1 / 4, RTrigger1 / 4, -RTrigger1 / 4, RTrigger1 / 4);
            } else if (LTrigger1 > 0.05) {
                SetPower(LTrigger1 / 4, -LTrigger1 / 4, LTrigger1 / 4, -LTrigger1 / 4);
            } else if (LBumper1) {
                SetPower(0.5, -0.5, -0.5, 0.5);
            } else if (RBumper1) {
                SetPower(-0.5, 0.5, 0.5, -0.5);
            } else if (dpadUp1) {
                SetPower(-0.2, -0.2, -0.2, -0.2);
            } else if (dpadDown1) {
                SetPower(0.2, 0.2, 0.2, 0.2);
            } else if (dpadLeft1) {
                SetPower(0.2, -0.2, -0.2, 0.2);
            } else if (dpadRight1) {
                SetPower(-0.2, 0.2, 0.2, -0.2);
            } else {
                SetPower(0, 0, 0, 0);
                start = false;
            }


            if (!fixClaw) {
                if (dpadUp2) {
                    linearSlide.setTargetPosition(slideHigh);
                    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlide.setPower(slideFast);
                } else if (x2) {
                    linearSlide.setTargetPosition(beaconHeight);
                    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlide.setPower(slideFast);

                } else if (dpadRight2) {
                    linearSlide.setTargetPosition(slideLow);
                    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlide.setPower(slideFast);
                } else if (dpadLeft2) {

                    linearSlide.setTargetPosition(coneHeights[currentCone - 1]);
                    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlide.setPower(1.0);

                    if(!coneFallEdge) {
                        currentCone -= 1;
                        coneFallEdge = true;
                    }

                    if(currentCone <= 0) currentCone = 5;
                } else if (dpadDown2) {
                    //if(!detectJunction()) { TRUST JERRY
                    if (true) {
                        linearSlide.setTargetPosition(slideBottom);
                        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        linearSlide.setPower(slideSlow);
                        flashLED = false;
                    } else {
                        flashLED = true;
                    }
                } else if (LBumper2 && linearSlide.getCurrentPosition() >= slideHigh - 200) {
//                    linearSlide.setTargetPosition(linearSlide.getCurrentPosition() - 150);
                    linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlide.setPower(-slideFast);
                } else if ((RBumper2 && linearSlide.getCurrentPosition() <= 0)) {
//                    linearSlide.setTargetPosition(linearSlide.getCurrentPosition() + 150
//                    );
                    linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlide.setPower(slideFast);
                } else if (RTrigger2 >= .1) {
                    linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlide.setPower(.4);

                } else if (y2) {
                    linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlide.setPower(0);
                } else if (flashLED && a1) { //Override
                    linearSlide.setTargetPosition(slideBottom);
                    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlide.setPower(slideFast);
                    flashLED = false;
                } else {
                    if (linearSlide.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        if (linearSlide.getCurrentPosition() < -200) {
                            linearSlide.setTargetPosition(linearSlide.getCurrentPosition());
                            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            linearSlide.setPower(.1);
                        } else {
                            linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            linearSlide.setPower(0);
                        }
                    } else if (linearSlide.getMode() == DcMotor.RunMode.RUN_TO_POSITION && linearSlide.getTargetPosition() == 0 && linearSlide.getCurrentPosition() > -10) {
                        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        linearSlide.setPower(0);
                    }
                }
            }
            else {
                //Fix Claw
                if(limit.isPressed()) {
                    fixClaw = false;
                    linearSlide.setPower(0);
                    linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else {
                    linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    linearSlide.setPower(slideSlow/2);
                }
            }
            if(!dpadLeft2) {
                coneFallEdge = false;
            }


            if (a2){
                clawPos = clawOpen;
            } if(b2) {
                clawPos = clawGrab;
            }

            if(square_state == 1 && !x2){
                square_state = 0;
                clawPos = clawGrab;
            }
            claw.setPosition(clawPos);


            if(flipper_status == 0  && linearSlide.getCurrentPosition() < -1250) {
                flipper.setPosition(flipperUp);
            } else if (flipper_status == 2 && linearSlide.getCurrentPosition() < -1250) {
                flipper.setPosition(flipperDown);
            }

            //Code to switch AFTER stick released
            if(flipper_status == 0 && LStickIn2) {
                flipper_status = 3;
            }
            if(flipper_status == 2 && LStickIn2) {
                flipper_status = 1;
            }
            if(!LStickIn2 && flipper_status % 2 == 1) { //1 or 3
                flipper_status -= 1;
            }

            //color to say if we're close enough to cone

            if(x1) {
                fixClaw = false;
                dist = false;
            }

            if(dist) {
                if (clawPos == clawOpen && distanceF.getDistance(DistanceUnit.MM) < 82 || distanceH.getDistance(DistanceUnit.INCH) < 2.4) {
                    redLED.setState(false);
                    greenLED.setState(false);
                } else if (clawPos == clawOpen) {
                    redLED.setState(true);
                    greenLED.setState(false);
                } else {
                    redLED.setState(false);
                    greenLED.setState(true);
                }
            } else {
                if(clawPos == clawOpen) {
                    redLED.setState(true);
                    greenLED.setState(false);
                } else {
                    redLED.setState(false);
                    greenLED.setState(true);
                }
            }

            if(RStickIn2){
                linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


            telemetry.addData("CONE STACK - CURRENT CONE: \n", currentCone);
//            telemetry.addData("claw", claw.getPosition());
//            if(dist) {
//                telemetry.addData("F Distance", distanceF.getDistance(DistanceUnit.MM));
//                telemetry.addData("H Distance", distanceH.getDistance(DistanceUnit.MM));
//            }
//            telemetry.addData("limit switch", limitSwitch.isPressed());
//            telemetry.addData("slide encoder", linearSlide.getCurrentPosition());
//           telemetry.addData("GY", imu.getAngularOrientation().firstAngle);
//           telemetry.addData("AVG", getAvgHeading());
//            telemetry.addData( "kb: " , (double)(Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1024);
            telemetry.update();



        }
    }
}
