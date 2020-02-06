package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public abstract class Bendy extends LinearOpMode {
	private static DcMotor[] motors = new DcMotor[7];
	private static Servo[] servos = new Servo[5];
	//private double[] velocity = {0,0,0,0,0,0,0};
	private static String[] names = {"a", "b", "x", "y"};
	private static boolean[] prevButtons = {false, false, false, false};
	private static BNO055IMU imu;
	private static boolean clamped = false;
	
	@Override
	public abstract void runOpMode();
//	Bendy(HardwareMap hardwareMap) {
//		map = hardwareMap;
//		motors[0] = map.get(DcMotor.class, "leftFront");
//		motors[1] = map.get(DcMotor.class, "rightFront");
//		motors[2] = map.get(DcMotor.class, "rightBack");
//		motors[3] = map.get(DcMotor.class, "leftBack");
//		motors[4] = map.get(DcMotor.class, "lift");
//		motors[5] = map.get(DcMotor.class, "lift2");
//		motors[6] = map.get(DcMotor.class, "slide");
//
//		servos[0] = map.get(Servo.class, "clamp");
//		servos[1] = map.get(Servo.class, "clamp2");
//		servos[2] = map.get(Servo.class, "rotate");
//		servos[3] = map.get(Servo.class, "rotate2");
//		servos[4] = map.get(Servo.class, "pull");
//
//		for(int i = 0; i < motors.length; i++) {
//			motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//			if(i == 1 || i == 2) motors[i].setDirection(DcMotor.Direction.REVERSE);
//			motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//			motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//		}
//
//		BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
//		parameters2.mode = BNO055IMU.SensorMode.IMU;
//		parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//		parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//		parameters2.loggingEnabled = false;
//
//		imu = hardwareMap.get(BNO055IMU.class, "imu");
//		imu.initialize(parameters2);
//	}

	void setUp() {
		motors[0] = hardwareMap.get(DcMotor.class, "leftFront");
		motors[1] = hardwareMap.get(DcMotor.class, "rightFront");
		motors[2] = hardwareMap.get(DcMotor.class, "rightBack");
		motors[3] = hardwareMap.get(DcMotor.class, "leftBack");
		motors[4] = hardwareMap.get(DcMotor.class, "lift");
		motors[5] = hardwareMap.get(DcMotor.class, "lift2");
		motors[6] = hardwareMap.get(DcMotor.class, "slide");

		servos[0] = hardwareMap.get(Servo.class, "clamp");
		servos[1] = hardwareMap.get(Servo.class, "clamp2");
		servos[2] = hardwareMap.get(Servo.class, "rotate");
		servos[3] = hardwareMap.get(Servo.class, "rotate2");
		servos[4] = hardwareMap.get(Servo.class, "pull");

		for(int i = 0; i < motors.length; i++) {
			motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			if(i == 1 || i == 2) motors[i].setDirection(DcMotor.Direction.REVERSE);
			motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}

		BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
		parameters2.mode = BNO055IMU.SensorMode.IMU;
		parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters2.loggingEnabled = false;

		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters2);
	}

	private double constrain(double value, double min, double max) {
		return (value < max) ? ((value > min) ? value : min) : max;
	}




	/*private void setVelocity(int index, double newVelocity, Telemetry telemetry) {
		velocity[index] = constrain(velocity[index], -1, 1);
		if(velocity[index] < constrain(newVelocity,-1,1)) {
			velocity[index] += 0.05;
		}else if(velocity[index] > constrain(newVelocity,-1,1)) {
			velocity[index] -= 0.05;
		}
		velocity[index] = (double)Math.round(velocity[index] * 100) / 100;
		motors[index].setPower(velocity[index]);
		telemetry.addData("bad", velocity[index]);
	}*/

	public void drive(double angle, double power) {
		motors[0].setPower(power * Math.sin(Math.toRadians(angle)));
		motors[1].setPower(power * Math.cos(Math.toRadians(angle)));
		motors[2].setPower(power * Math.sin(Math.toRadians(angle)));
		motors[3].setPower(power * Math.cos(Math.toRadians(angle)));
	}

	public void drive(double y, double x, double turn) {
		double speed = Math.hypot(y, x);
		double robotAngle = Math.atan2(y, x) - Math.PI / 4;

		// do {
		motors[0].setPower(speed * Math.sin(robotAngle) - turn);
		motors[1].setPower(speed * Math.cos(robotAngle) + turn);
		motors[2].setPower(speed * Math.sin(robotAngle) + turn);
		motors[3].setPower(speed * Math.cos(robotAngle) - turn);
		// } while(Math.abs(constrain(velocity[0],-1,1) - constrain(speed * Math.sin(robotAngle) - turn,-1,1)) > 0.05 && !teleop);
	}

	public void drive(double y, double x, double turn, int position) {
		double robotAngle = Math.atan2(y, x) - Math.PI / 4;

		motors[0].setTargetPosition((int)(-position * constrain(Math.sin(robotAngle) - turn, -1, 1)) + motors[0].getCurrentPosition());
		motors[1].setTargetPosition((int)(-position * constrain(Math.cos(robotAngle) + turn, -1, 1)) + motors[1].getCurrentPosition());
		motors[2].setTargetPosition((int)(-position * constrain(Math.sin(robotAngle) + turn, -1, 1)) + motors[2].getCurrentPosition());
		motors[3].setTargetPosition((int)(-position * constrain(Math.cos(robotAngle) - turn, -1, 1)) + motors[3].getCurrentPosition());
		for(int i = 0; i < 4; i++) {
			motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
		drive(y,x,turn);

		// boolean[] motorsBusy = {true, true, true, true};

		// while(motorsBusy[0] || motorsBusy[1] || motorsBusy[2] || motorsBusy[3]) {
		// 	for(int i = 0; i < 4; i++) {
		// 		if(motorsBusy[i]) motorsBusy[i] = motors[i].isBusy();
		// 	}
		// }
	}

	public void drive(double angle, double power, int position) {
		motors[0].setTargetPosition((int)(-position * Math.sin(Math.toRadians(angle)) + motors[0].getCurrentPosition()));
		motors[1].setTargetPosition((int)(-position * Math.cos(Math.toRadians(angle)) + motors[1].getCurrentPosition()));
		motors[2].setTargetPosition((int)(-position * Math.sin(Math.toRadians(angle)) + motors[2].getCurrentPosition()));
		motors[3].setTargetPosition((int)(-position * Math.cos(Math.toRadians(angle)) + motors[3].getCurrentPosition()));
		for(int i = 0; i < 4; i++) {
			motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
		drive(angle, power);
	}

	public void encoderStop() {
		for(int i = 0; i < 4; i++) {
			while(motors[i].isBusy());
			motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			motors[i].setPower(0);
		}
	}

	public void encoderStop(String name) {
		DcMotor motor = hardwareMap.get(DcMotor.class, name);
		for(int i = 0; i < 4; i++) {
			while(motors[i].isBusy());
			motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			motors[i].setPower(0);
		}
	}

	public void move(String name, double power, int position) {
		DcMotor motor = hardwareMap.get(DcMotor.class, name);
		motor.setTargetPosition(position + motor.getCurrentPosition());
		motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor.setPower(power);
	}

	public boolean toggleButton(String name, boolean button) {
		int index = 0;
		for(int i = 0; i < names.length; i++) {
			if(names[i].equals(name)) {
				index = i;
			}
		}
		boolean prevValue = prevButtons[index];
		prevButtons[index] = button;
		return button && ! prevButtons[index];
	}

	public void gyroAlign(double position) {
		double gyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
		while(Math.abs(gyro - position) > 2 && opModeIsActive()) {
			gyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
			drive(0, 0, (gyro - position) / 180);
		}
	}

	public void gyroCalibrate() {
		while (!imu.isGyroCalibrated() && !isStopRequested()) {
			sleep(20);
			idle();
		}
	}

	public void clamp() {
		servos[0].setPosition(clamped ? 1 : 0);
		servos[1].setPosition(clamped ? 0 : 1);
		clamped = !clamped;
	}
}
