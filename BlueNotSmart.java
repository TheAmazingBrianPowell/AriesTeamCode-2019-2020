package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous
public class BlueNotSmart extends LinearOpMode {
	private DcMotor leftBack, rightBack, leftFront, rightFront, slide, lift, lift2;
	private ElapsedTime runtime = new ElapsedTime();

	@Override
	public void runOpMode() {
		Servo clamp, clamp2, rotate, rotate2, pull, pull2;
		ColorSensor frontColor, groundColor;
		DistanceSensor frontDistance;
		//motors
		leftBack = hardwareMap.get(DcMotor.class, "leftBack");
		leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftBack.setDirection(DcMotor.Direction.FORWARD);
		leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		rightBack = hardwareMap.get(DcMotor.class, "rightBack");
		rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightBack.setDirection(DcMotor.Direction.REVERSE);
		rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		leftFront = hardwareMap.get(DcMotor.class, "leftFront");
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setDirection(DcMotor.Direction.FORWARD);
		leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		rightFront = hardwareMap.get(DcMotor.class, "rightFront");
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFront.setDirection(DcMotor.Direction.REVERSE);
		leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		//Linear slider for moving intake and outtake system
		slide = hardwareMap.get(DcMotor.class, "slide");
		slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		slide.setDirection(DcMotor.Direction.REVERSE);

		//linear sliders
		lift = hardwareMap.get(DcMotor.class, "lift");
		lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift.setDirection(DcMotor.Direction.REVERSE);
		lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		lift2 = hardwareMap.get(DcMotor.class, "lift2");
		lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift2.setDirection(DcMotor.Direction.FORWARD);
		lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		//Servos for intake control
		clamp = hardwareMap.get(Servo.class, "clamp");
		clamp2 = hardwareMap.get(Servo.class, "clamp2");
		pull = hardwareMap.get(Servo.class, "pull");
		pull2 = hardwareMap.get(Servo.class, "pull2");
		rotate = hardwareMap.get(Servo.class, "rotate");
		rotate2 = hardwareMap.get(Servo.class, "rotate2");

		//color/distance sensor on the front of robot for sensing stones' distances from the robot
		frontColor = hardwareMap.get(ColorSensor.class, "oof");
		frontDistance = hardwareMap.get(DistanceSensor.class, "oof");
		groundColor = hardwareMap.get(ColorSensor.class, "colorSensor");
		waitForStart();
		runtime.reset();

		if (opModeIsActive()) {
			pull.setPosition(0);
			pull2.setPosition(0);
			encoderDrive( - 0.7, -0.7, -0.7, -0.7, 500);

			//thingy
			encoderDrive(0.7, -0.7, -0.7, 0.7, 750);

			encoderDrive( - 0.2, -0.2, -0.2, -0.2, 1000);
			pull.setPosition(1);
			pull2.setPosition(1);
			sleep(1000);
			encoderDrive(0.3, 0.3, 0.3, 0.3, 2000);
			pull.setPosition(0);
			pull2.setPosition(0);
			sleep(1000);
			encoderDrive( - 0.2, -0.2, -0.2, -0.2, 80);
			//thingy
			encoderDrive( - 0.7, 0.7, 0.7, -0.7, 1550);
			encoderDrive( - 0.7, -0.7, -0.7, -0.7, 900);
			rotate.setPosition(0);
			rotate2.setPosition(1);
			sleep(2000);
			rotate.setPosition(0.5);
			rotate.setPosition(0.5);
			encoderDrive( - 0.7, 0.7, 0.7, -0.7, 2400);

			encoderDrive(0.2, 0.2, 0.2, 0.2, 100);

			clamp.setPosition(0);
			clamp2.setPosition(1);
			encoderDrive( - 1, 1, -1, 1, 1730);

			encoderDrive(0.2, 0.2, 0.2, 0.2, 500);

			clamp.setPosition(1);
			clamp2.setPosition(0);
			sleep(1000);

			encoderDrive( - 0.2, -0.2, -0.2, -0.2, 600);

			encoderDrive( - 1, 1, -1, 1, 800);

			encoderDrive(1, 1, 1, 1, 1700);

			lift.setPower( - 0.3);
			lift2.setPower( - 0.3);
			sleep(1000);
			lift.setPower(0);
			lift2.setPower(0);

			encoderDrive(1, 1, 1, 1, 700);

			clamp.setPosition(0);
			clamp2.setPosition(1);

			sleep(500);

			encoderDrive( - 1, -1, -1, -1, 800);

			lift.setPower(0.3);
			lift2.setPower(0.3);
			sleep(900);

			// encoderDrive(0.2,0.2,0.2,0.2,1000);
			// encoderDrive(0.2,-0.2,-0.2,0.2,7000);
			// encoderDrive(-0.2,-0.2,-0.2,-0.2,10000);
			//  // encoderDrive(-0.2,0.2,0.2,-0.2, 13000);
			//  encoderDrive(-0.2,-0.2,-0.2,-0.2,15000);
			//  encoderDrive(-0.2,0.2,0.2,-0.2, 6000);
			//  encoderDrive(0.5,0.5,0.5,0.5,4000);
			//  encoderDrive(0.2,-0.2,-0.2,0.2, 12000);
		}
	}
	private void move(double y, double x, double turn) {
		double speed = Math.hypot(y, x);
		double robotAngle = -Math.atan2(y, x) - Math.PI / 4;

		leftFront.setPower(speed * Math.sin(robotAngle) + turn);
		rightFront.setPower(speed * Math.cos(robotAngle) - turn);
		leftBack.setPower(speed * Math.cos(robotAngle) + turn);
		rightBack.setPower(speed * Math.sin(robotAngle) - turn);
	}
	private void move(double angle, double power) {
		leftFront.setPower(power * Math.sin(Math.toRadians(angle)));
		rightFront.setPower(power * Math.cos(Math.toRadians(angle)));
		leftBack.setPower(power * Math.cos(Math.toRadians(angle)));
		rightBack.setPower(power * Math.sin(Math.toRadians(angle)));
	}
	private void encoderDrive(double power1, double power2, double power3, double power4, double distance) {

		leftFront.setTargetPosition((int) Math.round(distance * -Math.abs(power1) / power1) + leftFront.getCurrentPosition());
		rightFront.setTargetPosition((int) Math.round(distance * -Math.abs(power2) / power2) + rightFront.getCurrentPosition());
		leftBack.setTargetPosition((int) Math.round(distance * -Math.abs(power3) / power3) + leftBack.getCurrentPosition());
		rightBack.setTargetPosition((int) Math.round(distance * -Math.abs(power4) / power4) + rightBack.getCurrentPosition());

		//RUN_TO_POSITION allows the robots motors to remain “busy” until they finish running to a certain point
		leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		leftFront.setPower( - power1);
		rightFront.setPower( - power2);
		leftBack.setPower( - power3);
		rightBack.setPower( - power4);
		//etc…
		while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy());

		//motors have reached their positions, set their power to zero
		leftFront.setPower(0);
		rightFront.setPower(0);
		leftBack.setPower(0);
		rightBack.setPower(0);

		//run with encoder only uses the encoders to regulate the speed of the motors
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}
}
