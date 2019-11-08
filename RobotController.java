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
   int speedAdjust = 8;

   @Override
   public void runOpMode() {
       //create variables
       ElapsedTime runtime = new ElapsedTime();
       boolean clamped = false, isPulling = false, bChanged = false, xChanged = false, dpadLeft = false, dpadRight = false, prevLeft = false, prevRight = false;
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
       DcMotor lift = hardwareMap.get(DcMotor.class, "lift");
       lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       lift.setDirection(DcMotor.Direction.REVERSE);
       lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
       DcMotor lift2 = hardwareMap.get(DcMotor.class, "lift2");
       lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       lift2.setDirection(DcMotor.Direction.FORWARD);
       lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
       //Bigger linear slider
       DcMotor slide2 = hardwareMap.get(DcMotor.class, "slide2");
       slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       slide2.setDirection(DcMotor.Direction.REVERSE);
       slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       //Servo
       Servo clamp1 = hardwareMap.get(Servo.class, "rotate");
       Servo clamp2 = hardwareMap.get(Servo.class, "clamp2"); //<-- oops
       Servo pull = hardwareMap.get(Servo.class, "pull");
       Servo rotate = hardwareMap.get(Servo.class, "clamp1");

       // wait for the start button
       waitForStart();
       runtime.reset();

       while (opModeIsActive()) {
           //--------------------------------------------------------------------------------------
           // controls linear slider
           //--------------------------------------------------------------------------------------
           if(gamepad1.dpad_up && lift2.getCurrentPosition() > -800) {
               lift.setPower(-0.1);
               lift2.setPower(-0.1);
           }
           else if(gamepad1.dpad_down && lift2.getCurrentPosition() < 1) {
               lift.setPower(0.1);
               lift2.setPower(0.1);
           }else{
               lift.setPower(0);
               lift2.setPower(0);
           }
           
           if(gamepad1.left_bumper && slide2.getCurrentPosition() < 0) {
               if(slide2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                   slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               }
               slide2.setPower(1);
           }
           else if(gamepad1.right_bumper && slide2.getCurrentPosition() > -8800) {
               if(slide2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                   slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
               }
               slide2.setPower(-1);
           }
           else if(((!gamepad1.left_bumper && !gamepad1.right_bumper) || (slide2.getCurrentPosition() > 0 || slide2.getCurrentPosition() < -8000)) && slide2.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
               slide2.setPower(0);
           }
            if(!slide2.isBusy() && slide2.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                slide2.setPower(0);
               slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           }
           if(gamepad1.y) {
               slide2.setTargetPosition(0);
               slide2.setPower(1);
               slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           }
           if(gamepad1.dpad_left && !prevLeft) speedAdjust--;
           if(gamepad1.dpad_right && !prevRight) speedAdjust++;
           prevLeft = gamepad1.dpad_left;
           prevRight = gamepad1.dpad_right;
           
           bChanged = gamepad1.b;
           telemetry.addData("speed adjust",speedAdjust);
           telemetry.update();

           //--------------------------------------------------------------------------------------
           // set clip that attaches to build plate
           //--------------------------------------------------------------------------------------
           if(gamepad1.x && !xChanged) {
               pull.setPosition(isPulling ? 0 : 1);
               isPulling = !isPulling;
           }
           xChanged = gamepad1.x;

           //--------------------------------------------------------------------------------------
           // Uses the hypotenus of left joystick and right joystick to calculate the magnitude (speed) of the robot
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
    private void encodersRun(DcMotor motor, DcMotor motor2, double power, int position) {
        motor.setPower(power * constrain(position,-1,1));
        motor2.setPower(power * constrain(position,-1,1));
        motor.setTargetPosition(position + motor.getCurrentPosition());
        motor2.setTargetPosition(position + motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(!motor.isBusy()) {
            motor.setPower(0);
            motor2.setPower(0);
        }
    }
   //squares the power for logarithmic controls
   private double logControl(double power) {
       return ((power < 0) ? -power * power : power * power) * (speedAdjust / 10);
   }
   private double constrain(double value, double min, double max) {
       return ((value > min) ? ((value < max) ? value : max) : min);
   }
}
