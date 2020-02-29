//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.Servo;
//// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//// import org.firstinspires.ftc.robotcore.external.navigation.Position;
//// import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//
//@TeleOp
//public class RobotController extends LinearOpMode {
//   /**
//    This program was written by Brian Powell from Aries, Team 15342 for the main TeleOp program
//    */
//   private int speedAdjust = 8;
//   //BNO055IMU imu, imu2;
//
//   @Override
//   public void runOpMode() {
//       ElapsedTime runtime = new ElapsedTime();
//       boolean isPulling = false, clamped = false, prevX = false, prevLeft = false, prevRight = false, prevB = false, rotated = false, prevA = false;
//       double speed, robotAngle, realign = 0;
//       DcMotor leftBack, rightBack, leftFront, rightFront, lift, lift2, slide;
//       Servo clamp, clamp2, rotate, pull, wheel;
//       int timer = 0;
//
//       //wheels
//       leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
//       leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       leftBack.setDirection(DcMotor.Direction.FORWARD);
//       leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//       rightBack = hardwareMap.get(DcMotor.class, "rightBack");
//       rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       rightBack.setDirection(DcMotor.Direction.REVERSE);
//       rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//       leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
//       leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       leftFront.setDirection(DcMotor.Direction.FORWARD);
//       leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//       rightFront = hardwareMap.get(DcMotor.class, "rightFront");
//       rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       rightFront.setDirection(DcMotor.Direction.REVERSE);
//       rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//       //linear sliders
//       lift = hardwareMap.get(DcMotor.class, "lift");
//       lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       lift.setDirection(DcMotor.Direction.REVERSE);
//       lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//       lift2 = hardwareMap.get(DcMotor.class, "lift2");
//       lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       lift2.setDirection(DcMotor.Direction.FORWARD);
//       lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//       //x-rail linear slider
//       slide = hardwareMap.get(DcMotor.class, "slide");
//       slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       slide.setDirection(DcMotor.Direction.REVERSE);
//       slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//       slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//       //servos
//        clamp = hardwareMap.get(Servo.class, "clamp");
//        clamp2 = hardwareMap.get(Servo.class, "clamp2");
//        pull = hardwareMap.get(Servo.class, "pull");
//        //rotate = hardwareMap.get(Servo.class, "rotate");
//        //wheel = hardwareMap.get(Servo.class, "wheel");
//
//        // BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        // parameters.mode = BNO055IMU.SensorMode.IMU;
//        // parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        // parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        // parameters.loggingEnabled = false;
//
//       //  imu = hardwareMap.get(BNO055IMU.class, "imu");
//       //  imu.initialize(parameters);
//
//       //  imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
//       //  imu2.initialize(parameters);
//
//       // // wait for the start button and wait for the gyro sensor to calibrate
//       // while (!isStopRequested() && !imu.isGyroCalibrated()) {
//       //      sleep(50);
//       //      idle();
//       //  }
//        waitForStart();
//       runtime.reset();
//
//       while (opModeIsActive()) {
//           //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//           //Orientation angles2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
//            //telemetry.addData("angle", angles.firstAngle);
//           //prints the speed to the console
//           //telemetry.addData("Speed Adjust",speedAdjust);
//           telemetry.addData("encoders", leftFront.getCurrentPosition() + "," + rightFront.getCurrentPosition() + "," + rightBack.getCurrentPosition() + "," + leftBack.getCurrentPosition());
//
//           //telemetry.addData("rotateThingy", (float)lift2.getCurrentPosition() / -800.f);
//
//           //controls the clamp intake
//           if(gamepad1.b && !prevB) {
//               //clamp.setPosition(clamped ? 0.65 : 1);
//               //clamp2.setPosition(clamped ? 1 : 0);
//               clamp.setPosition(clamped ? 1 : 0);
//               clamp2.setPosition(clamped ? 0 : 1);
//               clamped = !clamped;
//           }
//           prevB = gamepad1.b;
//
//           // if(gamepad1.a && !prevA) {
//           //     realign = -(angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2;
//           // }
//
//           // if(gamepad1.left_trigger > 0) {
//           //   wheel.setPosition(1);
//           // } else if(gamepad1.right_trigger > 0) {
//           //  wheel.setPosition(0);
//           // } else {
//           //  wheel.setPosition(0.5);
//           // }
//
//           //controls linear slider
//           if(gamepad1.dpad_up && lift2.getCurrentPosition() > -800) {
//               lift.setPower(-0.3);
//               lift2.setPower(-0.3);
//               if(lift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
//                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//               }
//           }
//           else if(gamepad1.dpad_down && lift2.getCurrentPosition() < 1) {
//               lift.setPower(0.3);
//               lift2.setPower(0.3);
//               if(lift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
//                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//               }
//           }else{
//               lift.setPower(0);
//               lift2.setPower(0);
//           }
//
//           //controls x-rail extension
//           if(gamepad1.left_bumper && slide.getCurrentPosition() < 0) {
//               if(slide.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
//                   slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//               }
//               slide.setPower(1);
//           }
//           else if(gamepad1.right_bumper && slide.getCurrentPosition() > -8800) {
//               if(slide.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
//                   slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//               }
//               slide.setPower(-1);
//           }
//           else if(((!gamepad1.left_bumper && !gamepad1.right_bumper) || (slide.getCurrentPosition() > 0 || slide.getCurrentPosition() < -8000)) && slide.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
//               slide.setPower(0);
//           }
//           if(!slide.isBusy() && slide.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
//               slide.setPower(0);
//               slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//           }
//           if(!lift2.isBusy() && lift.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
//             lift.setPower(0);
//             lift2.setPower(0);
//             lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//             lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//             timer = 0;
//           }
//           telemetry.addData("timer", timer);
//           if(gamepad1.y) {
//               slide.setTargetPosition(0);
//               slide.setPower(1);
//               slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//               if(timer == 0) timer++;
//           }
//           if(timer > 0) timer++;
//           if(timer > 10) {
//                lift.setTargetPosition(5);
//                lift2.setTargetPosition(5);
//                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift.setPower(0.3);
//                lift2.setPower(0.3);
//               }
//
//           //controls speed adjusting
//           if(gamepad1.dpad_left && !prevLeft) speedAdjust--;
//           if(gamepad1.dpad_right && !prevRight) speedAdjust++;
//           prevLeft = gamepad1.dpad_left;
//           prevRight = gamepad1.dpad_right;
//
//           //sets clip that attaches to the foundation
//           if(gamepad1.x && !prevX) {
//               pull.setPosition(isPulling ? 0 : 1);
//               isPulling = !isPulling;
//           }
//           prevX = gamepad1.x;
//
//
//           //uses the hypotenuse of left joystick and right joystick to calculate the speed of the robot
//           speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//
//           //finds the angle the robot is moving at
//           //robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (angles.firstAngle + Math.PI + angles2.firstAngle + Math.PI) / 2 - Math.PI / 4 + realign;
//           robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//           //finds the percent of power to each wheel and multiplies it by the speed
//           leftFront.setPower(speed * Math.sin(robotAngle) - gamepad1.right_stick_x);
//           rightFront.setPower(speed * Math.cos(robotAngle) + gamepad1.right_stick_x);
//           leftBack.setPower(speed * Math.cos(robotAngle) - gamepad1.right_stick_x);
//           rightBack.setPower(speed * Math.sin(robotAngle) + gamepad1.right_stick_x);
//           telemetry.update();
//       }
//   }
//}
