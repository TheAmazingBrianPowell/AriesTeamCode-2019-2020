package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Disabled
public abstract class Bendy extends LinearOpMode {
	DcMotor[] motors = new DcMotor[7];
	Servo[] servos = new Servo[6];
	//private double[] velocity = {0,0,0,0,0,0,0};
	private String[] names = {"a", "b", "x", "y"};
	private boolean[] prevButtons = {false, false, false, false};
	private BNO055IMU imu;
	private boolean clamped = false;

	private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
	private static final String LABEL_FIRST_ELEMENT = "Stone";
	private static final String LABEL_SECOND_ELEMENT = "Skystone";
	private static final String VUFORIA_KEY = "AbxWk0X/////AAABmbTHsmdPK0rWsHtl3bN7AfNWxaadLD6LGw0yvaK6Jk8EVl3jSPKMV/fht+xuCmYwDfcTu4T7KVtMRmLI8fezTp2sgVQ4J3/7GYGp/duLM3448Ir4ER1r4IoTPhdWXFRUS0V3F2TAgM4PT7KUd15dMOm6LqVwsOu1Msfgv7tjQuvl2Dc6k16VOE/IVcd9UK31Q9zx15cF3kVn5/y7PS+kKcOZPSUn8ghxiPVrU7x4/9QYx9HInyQ4b6rK+8+dPUkcPF2n+1EfrjXSkmCCQna2AZiTxv3ASgCOULQtYgjdGRetha6CJYOgZrT1xF+qUX+KM1s/QkyBU1tq3TC2JN6m/+zBMU+LeSx+ivhstDHnd5iP";
	private VuforiaLocalizer vuforia;
	private TFObjectDetector tfod;
	
	ColorSensor groundColor;

	@Override
	public abstract void runOpMode();

	void setUp() {
		if(!isStopRequested()) {
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
			servos[5] = hardwareMap.get(Servo.class, "autoarm");
		}

		for(int i = 0; i < motors.length; i++) {
			if(!isStopRequested()) {
				if(i > 3) motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				if(i == 1 || i == 2) motors[i].setDirection(DcMotor.Direction.REVERSE);
				motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			} else {
				break;
			}
		}

		if(!isStopRequested()) {

			BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
			parameters2.mode = BNO055IMU.SensorMode.IMU;
			parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
			parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
			parameters2.loggingEnabled = false;

			imu = hardwareMap.get(BNO055IMU.class, "imu");
			imu.initialize(parameters2);

			groundColor = hardwareMap.get(ColorSensor.class, "colorSensor");
		}
	}

/*	void vuforiaInit() {
		float phoneXRotate	= 0;
		float phoneYRotate;
		float phoneZRotate	= 0;
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
		//VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = "AbxWk0X/////AAABmbTHsmdPK0rWsHtl3bN7AfNWxaadLD6LGw0yvaK6Jk8EVl3jSPKMV/fht+xuCmYwDfcTu4T7KVtMRmLI8fezTp2sgVQ4J3/7GYGp/duLM3448Ir4ER1r4IoTPhdWXFRUS0V3F2TAgM4PT7KUd15dMOm6LqVwsOu1Msfgv7tjQuvl2Dc6k16VOE/IVcd9UK31Q9zx15cF3kVn5/y7PS+kKcOZPSUn8ghxiPVrU7x4/9QYx9HInyQ4b6rK+8+dPUkcPF2n+1EfrjXSkmCCQna2AZiTxv3ASgCOULQtYgjdGRetha6CJYOgZrT1xF+qUX+KM1s/QkyBU1tq3TC2JN6m/+zBMU+LeSx+ivhstDHnd5iP";
		parameters.cameraDirection = CAMERA_CHOICE;
		parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

		if(!isStopRequested()) {
			vuforia = ClassFactory.getInstance().createVuforia(parameters);
		}

		VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

		if(!isStopRequested()) {

			skystone = targetsSkyStone.get(0);
			skystone.setName("Stone Target");

			skystone.setLocation(OpenGLMatrix.translation(0, 0, stoneZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
		}
		if(!isStopRequested()) {
			if (CAMERA_CHOICE == BACK) {
				phoneYRotate = -90;
			} else {
				phoneYRotate = 90;
			}

			if (PHONE_IS_PORTRAIT) {
				phoneXRotate = 90;
			}

			final float CAMERA_FORWARD_DISPLACEMENT = 6.0f * mmPerInch;
			final float CAMERA_VERTICAL_DISPLACEMENT = 3.0f * mmPerInch;
			final float CAMERA_LEFT_DISPLACEMENT = 0;

			OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

			((VuforiaTrackableDefaultListener)skystone.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
			targetsSkyStone.activate();
		}
	}*/
	
	void initTensorFlow() {
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		parameters.vuforiaLicenseKey = VUFORIA_KEY;
		parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

		vuforia = ClassFactory.getInstance().createVuforia(parameters);
		int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfodParameters.minimumConfidence = 0.8;
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
		if (tfod != null) {
			tfod.activate();
		}
	}

	/*boolean vuforiaSkystone() {
		if(!isStopRequested()) {
			return (((VuforiaTrackableDefaultListener)skystone.getListener()).isVisible());
		} else {
			return false;
		}
	}*/

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
		if(!isStopRequested()) {
			motors[0].setPower(power * Math.sin(Math.toRadians(angle)));
			motors[1].setPower(power * Math.cos(Math.toRadians(angle)));
			motors[2].setPower(power * Math.sin(Math.toRadians(angle)));
			motors[3].setPower(power * Math.cos(Math.toRadians(angle)));
		}
	}

	void drive(double y, double x, double turn) {
		if(!isStopRequested()) {
			//uses the hypotenuse of left joystick and right joystick to calculate the speed of the robot
			double speed = Math.hypot(x, y);

			//finds the angle the robot is moving at
			double robotAngle = Math.atan2(y, -x) - Math.PI / 4;
			
			motors[3].setPower(speed * Math.sin(robotAngle) - turn);
			motors[2].setPower(speed * Math.cos(robotAngle) + turn);
			motors[0].setPower(speed * Math.cos(robotAngle) - turn);
			motors[1].setPower(speed * Math.sin(robotAngle) + turn);
		}
	}

	void drive(double y, double x, double turn, int position) {
		if(!isStopRequested()) {
			double robotAngle = Math.atan2(y, x) - Math.PI / 4;
			double power = Math.hypot(y,x);
			int target0 = (int)(-position * constrain(Math.sin(robotAngle) - turn, -1, 1)) + motors[0].getCurrentPosition();
			int target1 = (int)(-position * constrain(Math.cos(robotAngle) + turn, -1, 1)) + motors[0].getCurrentPosition();
			int target2 = (int)(-position * constrain(Math.sin(robotAngle) + turn, -1, 1)) + motors[0].getCurrentPosition();
			int target3 = (int)(-position * constrain(Math.cos(robotAngle) - turn, -1, 1)) + motors[0].getCurrentPosition();

			// motors[0].setTargetPosition((int)(-position * constrain(Math.sin(robotAngle) - turn, -1, 1)) + motors[0].getCurrentPosition());
			// motors[1].setTargetPosition((int)(-position * constrain(Math.cos(robotAngle) + turn, -1, 1)) + motors[1].getCurrentPosition());
			// motors[2].setTargetPosition((int)(-position * constrain(Math.sin(robotAngle) + turn, -1, 1)) + motors[2].getCurrentPosition());
			// motors[3].setTargetPosition((int)(-position * constrain(Math.cos(robotAngle) - turn, -1, 1)) + motors[3].getCurrentPosition());

			//drive(y,x,turn);

			while(Math.abs(motors[0].getCurrentPosition() - target0) > 2 && Math.abs(motors[1].getCurrentPosition() - target1) > 2 && Math.abs(motors[2].getCurrentPosition() - target2) > 2 && Math.abs(motors[3].getCurrentPosition() - target3) > 2 && opModeIsActive()) {
				motors[0].setPower((double)(motors[0].getCurrentPosition() - target0) / 1500d * power);
				motors[1].setPower((double)(motors[1].getCurrentPosition() - target1) / 1500d * power);
				motors[2].setPower((double)(motors[2].getCurrentPosition() - target2) / 1500d * power);
				motors[3].setPower((double)(motors[3].getCurrentPosition() - target3) / 1500d * power);
				idle();
			}
			drive(0,0,0);

			// boolean[] motorsBusy = {true, true, true, true};

			// while(motorsBusy[0] || motorsBusy[1] || motorsBusy[2] || motorsBusy[3]) {
			// 	for(int i = 0; i < 4; i++) {
			// 		if(motorsBusy[i]) motorsBusy[i] = motors[i].isBusy();
			// 	}
			// }
		}
	}
	
	void drive(double y, double x, double turn, int position, int runAngle) {
		if(!isStopRequested()) {
			double robotAngle = Math.atan2(y, x) - Math.PI / 4;
			double power = Math.hypot(y,x);
			int target0 = (int)(-position * constrain(Math.sin(robotAngle) - turn, -1, 1)) + motors[0].getCurrentPosition();
			int target1 = (int)(-position * constrain(Math.cos(robotAngle) + turn, -1, 1)) + motors[0].getCurrentPosition();
			int target2 = (int)(-position * constrain(Math.sin(robotAngle) + turn, -1, 1)) + motors[0].getCurrentPosition();
			int target3 = (int)(-position * constrain(Math.cos(robotAngle) - turn, -1, 1)) + motors[0].getCurrentPosition();
			// motors[0].setTargetPosition((int)(-position * constrain(Math.sin(robotAngle) - turn, -1, 1)) + motors[0].getCurrentPosition());
			// motors[1].setTargetPosition((int)(-position * constrain(Math.cos(robotAngle) + turn, -1, 1)) + motors[1].getCurrentPosition());
			// motors[2].setTargetPosition((int)(-position * constrain(Math.sin(robotAngle) + turn, -1, 1)) + motors[2].getCurrentPosition());
			// motors[3].setTargetPosition((int)(-position * constrain(Math.cos(robotAngle) - turn, -1, 1)) + motors[3].getCurrentPosition());

			//drive(y,x,turn);

			while(Math.abs(motors[0].getCurrentPosition() - target0) > 10 && opModeIsActive()) {
				double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
				if(motors[0].getCurrentPosition() - target0 > 0) {
					motors[0].setPower((Math.abs(motors[0].getCurrentPosition() - target0) > 200 ? power : Math.abs(power) / power * 0.2) * target0  - (angle - runAngle) / 180d);
					motors[1].setPower((Math.abs(motors[0].getCurrentPosition() - target0) > 200 ? power : Math.abs(power) / power * 0.2) * target1 + (angle - runAngle) / 180d);
					motors[2].setPower((Math.abs(motors[0].getCurrentPosition() - target0) > 200 ? power : Math.abs(power) / power * 0.2) * target2 + (angle - runAngle) / 180d);
					motors[3].setPower((Math.abs(motors[0].getCurrentPosition() - target0) > 200 ? power : Math.abs(power) / power * 0.2) * target3 - (angle - runAngle) / 180d);
					idle();
				} else {
					motors[0].setPower((Math.abs(motors[0].getCurrentPosition() - target0) > 200 ? -power : -Math.abs(power) / power * 0.2) * target0 - (angle - runAngle) / 180d);
					motors[1].setPower((Math.abs(motors[0].getCurrentPosition() - target0) > 200 ? -power : -Math.abs(power) / power * 0.2) * target1 + (angle - runAngle) / 180d);
					motors[2].setPower((Math.abs(motors[0].getCurrentPosition() - target0) > 200 ? -power : -Math.abs(power) / power * 0.2) * target2 + (angle - runAngle) / 180d);
					motors[3].setPower((Math.abs(motors[0].getCurrentPosition() - target0) > 200 ? -power : -Math.abs(power) / power * 0.2) * target3 * target3 - (angle - runAngle) / 180d);
				}
			}
			drive(0,0,0);

			// boolean[] motorsBusy = {true, true, true, true};

			// while(motorsBusy[0] || motorsBusy[1] || motorsBusy[2] || motorsBusy[3]) {
			// 	for(int i = 0; i < 4; i++) {
			// 		if(motorsBusy[i]) motorsBusy[i] = motors[i].isBusy();
			// 	}
			// }
		}
	}


	/*void encoderStop() {
		while(motors[0].isBusy() && motors[1].isBusy() && motors[2].isBusy() && motors[3].isBusy() && opModeIsActive() && motors[0].getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
			telemetry.addData("0", motors[0].getMode() == DcMotor.RunMode.RUN_TO_POSITION);
			telemetry.addData("1", motors[1].getMode() == DcMotor.RunMode.RUN_TO_POSITION);
			telemetry.addData("2", motors[2].getMode() == DcMotor.RunMode.RUN_TO_POSITION);
			telemetry.addData("3", motors[3].getMode() == DcMotor.RunMode.RUN_TO_POSITION);
			telemetry.addData("0", motors[0].isBusy());
			telemetry.addData("1", motors[1].isBusy());
			telemetry.addData("2", motors[2].isBusy());
			telemetry.addData("3", motors[3].isBusy());
			telemetry.update();
		}
		if(!opModeIsActive()) {
			for(int i = 0; i < 4; i++) {
				motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				motors[i].setPower(0);
			}
		}
	}*/

	void encoderStop(String name) {
		if(!isStopRequested()) {
			DcMotor motor = hardwareMap.get(DcMotor.class, name);
			while(motor.isBusy()) idle();
			motor.setPower(0);
		}
	}

	void setPosition(String name, double position) {
		if(!isStopRequested()) {
			Servo servo = hardwareMap.get(Servo.class, name);
			servo.setPosition(position);
		}
	}

	void move(String name, double power, int position) {
		if(!isStopRequested()) {
			DcMotor motor = hardwareMap.get(DcMotor.class, name);
			motor.setTargetPosition((int)(Math.abs(power) / power * position) + motor.getCurrentPosition());
			motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motor.setPower(power);
		}
	}

	public boolean toggleButton(String name, boolean button) {
		if(!isStopRequested()) {
			int index = 0;
			for(int i = 0; i < names.length; i++) {
				if(names[i].equals(name)) {
					index = i;
				}
			}
			boolean prevValue = prevButtons[index];
			prevButtons[index] = button;
			return button && !prevValue;
		} else {
			return false;
		}
	}


	void gyroAlign(double position) {
		if(!isStopRequested()) {
			double gyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
			while(Math.abs(gyro - position) > 4 && opModeIsActive()) {
				gyro = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
				if(Math.abs(gyro - position) < 180) {
					drive(0, 0, (gyro - position) / 45);
				} else {
					drive(0, 0, (position - gyro) / 45);
				}
			}
			drive(0,0,0);
		}
	}

	void gyroCalibrate() {
		if(!isStopRequested()) {
			while (!imu.isGyroCalibrated() && !isStopRequested()) {
				sleep(20);
				idle();
			}
		}
	}

	void clamp() {
		if(!isStopRequested()) {
			servos[0].setPosition(clamped ? 1 : 0);
			servos[1].setPosition(clamped ? 0 : 1);
			clamped = !clamped;
		}
	}
}
