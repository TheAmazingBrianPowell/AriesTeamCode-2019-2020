package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RobotController extends LinearOpMode {
    @Override
    public void runOpMode() {
        //create variables
        ElapsedTime runtime = new ElapsedTime();
        DcMotor leftBack;
        DcMotor leftFront;
        DcMotor rightBack;
        DcMotor rightFront;
        Servo clamp1;
        Servo clamp2;
        String mode = "rotate";
        double currentRotation = 0;
        int time = 0;

        //Set Up The Hardware
        //Left back motor
        leftBack  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        //Right back motor
        rightBack = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //Left back motor
        leftFront  = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        //Right back motor
        rightFront = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        //Servo


        //Wait for the start button
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if(gamepad1.a && time >= 2000) {
                if(mode.equals("rotate")) {
                    mode = "translate";
                } else {
                    mode = "rotate";
                }
                time = 0;
            }
            time++;
            if(mode.equals("rotate")) {
                //Uses the hypothesis of left joystick and right joystick to calculate the magnitude (speed) of the robot
                double radius = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

                //Uses arc tangent (inverse tangent) to find how we need to rotate the power of the robot in order to move in the direction we want it to
                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

                //Note: Math.pow(value, 2) may be a method for achieving logarithmic controls, but negative numbers will become positive!

                //multiply the direction we need to travel in by the speed to create the power to each wheel, and also the amount that we are turning to get the power of the robot’s wheels
                leftFront.setPower(logControl(radius * Math.cos(robotAngle) - gamepad1.right_stick_x));
                rightFront.setPower(logControl(radius * Math.sin(robotAngle) + gamepad1.right_stick_x));
                leftBack.setPower(logControl(radius * Math.sin(robotAngle) - gamepad1.right_stick_x));
                rightBack.setPower(logControl(radius * Math.cos(robotAngle) + gamepad1.right_stick_x));
                // leftBack.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
                // rightBack.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
                // leftFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
                // rightFront.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            } else {
                //Uses the hypothesis of left joystick and right joystick to calculate the magnitude (speed) of the robot
                double radius = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

                //Uses arc tangent (inverse tangent) to find how we need to rotate the power of the robot in order to move in the direction we want it to
                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                currentRotation += robotAngle / 100;
                //Note: Math.pow(value, 2) may be a method for achieving logarithmic controls, but negative numbers will become positive!

                //multiply the direction we need to travel in by the speed to create the power to each wheel, and also the amount that we are turning to get the power of the robot’s wheels
                leftFront.setPower(logControl(radius * Math.cos(currentRotation) - gamepad1.right_stick_x));
                rightFront.setPower(logControl(radius * Math.sin(currentRotation) + gamepad1.right_stick_x));
                leftBack.setPower(logControl(radius * Math.sin(currentRotation) - gamepad1.right_stick_x));
                rightBack.setPower(logControl(radius * Math.cos(currentRotation) + gamepad1.right_stick_x));
            }
        }
    }

    //squares the power for logritmic controls
    private double logControl(double power) {
        return ((power < 0) ? -power * power : power * power);
    }
}

