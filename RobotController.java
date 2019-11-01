package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RobotController extends LinearOpMode {
   /**
    This program was written by Brian Powell from Aries, Team 15342 for the main TeleOp program
    */
   double speedAdjust = 1;

   @Override
   public void runOpMode() {
       //create variables
       ElapsedTime runtime = new ElapsedTime();
       boolean clamped = false, isPulling = false, bChanged = false, xChanged = false, dpadLeft = false, dpadRight = false;

       //Set Up The Hardware
       //Left back motor
       DcMotor leftBack  = hardwareMap.get(DcMotor.class, "leftFront");
       leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       leftBack.setDirection(DcMotor.Direction.FORWARD);
       leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       //Right back motor
       DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightFront");
       rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       rightBack.setDirection(DcMotor.Direction.REVERSE);
       rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       //Left back motor
       DcMotor leftFront  = hardwareMap.get(DcMotor.class, "leftBack");
       leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       leftFront.setDirection(DcMotor.Direction.FORWARD);
       leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       //Right back motor
       DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightBack");
       rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       rightFront.setDirection(DcMotor.Direction.REVERSE);
       rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       //Linear slider
       DcMotor slide = hardwareMap.get(DcMotor.class, "slide");
       slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       slide.setDirection(DcMotor.Direction.REVERSE);

       //Servo
       Servo clamp1 = hardwareMap.get(Servo.class, "clamp1");
       Servo clamp2 = hardwareMap.get(Servo.class, "rotate"); //<-- oops
       Servo pull = hardwareMap.get(Servo.class, "pull");
       Servo rotate = hardwareMap.get(Servo.class, "clamp2");

       // wait for the start button
       waitForStart();
       runtime.reset();

       while (opModeIsActive()) {
           //--------------------------------------------------------------------------------------
           // controls linear slider
           //--------------------------------------------------------------------------------------
           if(gamepad1.dpad_up) slide.setPower(1);
           else if(gamepad1.dpad_down) slide.setPower(-1);
           else slide.setPower(0);

           //--------------------------------------------------------------------------------------
           // controls clamp for grabbing stones
           //--------------------------------------------------------------------------------------
           if(gamepad1.b && !bChanged) {
               clamp1.setPosition(clamped ? 0.5 : -1);
               clamp2.setPosition(clamped ? 0.1 : 0.5);
               clamped = !clamped;
           }
           bChanged = gamepad1.b;

           //--------------------------------------------------------------------------------------
           //rotates claw if the bumper buttons are pressed
           //--------------------------------------------------------------------------------------
           if(gamepad1.left_bumper) rotate.setPosition(-1);
           else if (gamepad1.right_bumper) rotate.setPosition(1);

           //--------------------------------------------------------------------------------------
           //rotates claw if the bumper buttons are pressed
           //--------------------------------------------------------------------------------------
           if(gamepad1.dpad_left && !dpadLeft && speedAdjust >= 0.5) speedAdjust-=0.5;
           else if (gamepad1.right_bumper && !dpadRight && speedAdjust <= 0.5) speedAdjust+=0.5;
           dpadLeft = gamepad1.dpad_left;
           dpadRight = gamepad1.dpad_right;

           //--------------------------------------------------------------------------------------
           // set clip that attaches to build plate
           //--------------------------------------------------------------------------------------
           if(gamepad1.x && !xChanged) {
               pull.setPosition(isPulling ? 0 : 1);
               isPulling = !isPulling;
           }
           xChanged = gamepad1.x;

           //--------------------------------------------------------------------------------------
           // Uses the hypothesis of left joystick and right joystick to calculate the magnitude (speed) of the robot
           //--------------------------------------------------------------------------------------
           double radius = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

           //--------------------------------------------------------------------------------------
           // Uses arc tangent (inverse tangent function) to find how we need to rotate the power of the robot in order to move in the direction we want it to
           //--------------------------------------------------------------------------------------
           double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;

           //--------------------------------------------------------------------------------------
           // multiply the direction we need to travel in by the speed to create the power to each wheel, and also the amount that we are turning to get the power of the robot’s wheels
           //--------------------------------------------------------------------------------------
           leftFront.setPower(logControl(radius * Math.cos(robotAngle) - gamepad1.right_stick_x));
           rightFront.setPower(logControl(radius * Math.sin(robotAngle) + gamepad1.right_stick_x));
           leftBack.setPower(logControl(radius * Math.sin(robotAngle) - gamepad1.right_stick_x));
           rightBack.setPower(logControl(radius * Math.cos(robotAngle) + gamepad1.right_stick_x));
       }
   }

   //squares the power for logarithmic controls
   private double logControl(double power) {
       return ((power < 0) ? -power * power : power * power) / speedAdjust;
   }
}
