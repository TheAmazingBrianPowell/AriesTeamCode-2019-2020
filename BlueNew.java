//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//
//import java.util.ArrayList;
//import java.util.List;
//
//import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
//import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//
//@Autonomous
//
//public class BlueNew extends LinearOpMode {
//	private DcMotor leftBack, rightBack, leftFront, rightFront;
//	private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
//	private static final boolean PHONE_IS_PORTRAIT = false ;
//
//	private static final float mmPerInch = 25.4f;
//	private static final float mmTargetHeight = (6) * mmPerInch;
//
//	private static final float stoneZ = 2.00f * mmPerInch;
//
//	private static final float halfField = 72 * mmPerInch;
//	private static final float quadField  = 36 * mmPerInch;
//
//	private OpenGLMatrix lastLocation = null;
//	private VuforiaLocalizer vuforia = null;
//	private boolean targetVisible = false;
//	private float phoneXRotate	= 0;
//	private float phoneYRotate	= 0;
//	private float phoneZRotate	= 0;
//	private BNO055IMU imu, imu2;
//
//	@Override
//	public void runOpMode() {
//		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//		//VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//		parameters.vuforiaLicenseKey = "";
//		parameters.cameraDirection = CAMERA_CHOICE;
//
//		//  Instantiate the Vuforia engine
//		vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//		VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
//
//		VuforiaTrackable skystone = targetsSkyStone.get(0);
//		skystone.setName("Stone Target");
//
//		skystone.setLocation(OpenGLMatrix.translation(0, 0, stoneZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//		if (CAMERA_CHOICE == BACK) {
//			phoneYRotate = -90;
//		} else {
//			phoneYRotate = 90;
//		}
//
//		if (PHONE_IS_PORTRAIT) {
//			phoneXRotate = 90;
//		}
//
//		// Next, translate the camera lens to where it is on the robot.
//		// In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
//		final float CAMERA_FORWARD_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
//		final float CAMERA_VERTICAL_DISPLACEMENT = 3.0f * mmPerInch;   // eg: Camera is 3 Inches above ground
//		final float CAMERA_LEFT_DISPLACEMENT = 0;	 // eg: Camera is ON the robot's center line
//
//		OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));
//
//		//Let all the trackable listeners know where the phone is.
//		((VuforiaTrackableDefaultListener)skystone.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
//		targetsSkyStone.activate();
//		DcMotor slide, lift, lift2;
//		boolean didFind = false;
//		ColorSensor frontColor, groundColor, frontColor2;
//		DistanceSensor frontDistance, frontDistance2;
//		Servo rotate, rotate2, clamp, clamp2, pull;
//		int skyLocation = 0;
//
//		//motors
//		leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
//		leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		leftBack.setDirection(DcMotor.Direction.FORWARD);
//		leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//		rightBack = hardwareMap.get(DcMotor.class, "rightBack");
//		rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		rightBack.setDirection(DcMotor.Direction.REVERSE);
//		rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//		leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
//		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		leftFront.setDirection(DcMotor.Direction.FORWARD);
//		leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//		rightFront = hardwareMap.get(DcMotor.class, "rightFront");
//		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		rightFront.setDirection(DcMotor.Direction.REVERSE);
//		rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//		//Linear slider for moving intake and outtake system
//		slide = hardwareMap.get(DcMotor.class, "slide");
//		slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		slide.setDirection(DcMotor.Direction.REVERSE);
//
//		//linear sliders
//		lift = hardwareMap.get(DcMotor.class, "lift");
//		lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		lift.setDirection(DcMotor.Direction.REVERSE);
//		lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//		lift2 = hardwareMap.get(DcMotor.class, "lift2");
//		lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		lift2.setDirection(DcMotor.Direction.FORWARD);
//		lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//		groundColor = hardwareMap.get(ColorSensor.class, "colorSensor");
//
//		rotate = hardwareMap.get(Servo.class, "rotate");
//		rotate2 = hardwareMap.get(Servo.class, "rotate2");
//		clamp = hardwareMap.get(Servo.class, "clamp");
//		clamp2 = hardwareMap.get(Servo.class, "clamp2");
//		pull = hardwareMap.get(Servo.class, "pull");
//
//		BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
//		parameters2.mode = BNO055IMU.SensorMode.IMU;
//		parameters2.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//		parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//		parameters2.loggingEnabled = false;
//
//		imu = hardwareMap.get(BNO055IMU.class, "imu");
//		imu.initialize(parameters2);
//
//		imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
//		imu2.initialize(parameters2);
//
//		while (!isStopRequested() && !imu.isGyroCalibrated()) {
//			sleep(50);
//			idle();
//		}
//		while(opModeIsActive()) {
//			telemetry.addData("hi",((VuforiaTrackableDefaultListener)skystone.getListener()).isVisible());
//			telemetry.update();
//		}
//
//		telemetry.addData("status", "Ready");
//		telemetry.update();
//		waitForStart();
//		encoderDrive(0.5,-0.5,-0.5,0.5,400);
//
//		// lift.setPower(0.3);
//		// lift2.setPower(0.3);
//		// sleep(800);
//		// lift.setPower(0);
//		// lift2.setPower(0);
//		encoderDrive(0.4,0.4,0.4,0.4,2300, 0);
//		sleep(1000);
//		if (!(((VuforiaTrackableDefaultListener)skystone.getListener()).isVisible())) {
//			encoderDrive(-0.5,0.5,0.5,-0.5,750, 0);
//			sleep(1000);
//			if (!(((VuforiaTrackableDefaultListener)skystone.getListener()).isVisible())) {
//				encoderDrive(-0.5,0.5,0.5,-0.5, 750, 0);
//			}
//		}
//		encoderDrive(-0.5,-0.5,-0.5,-0.5,800,0);
//		// if (frontColor.alpha() - frontColor2.alpha() >= 40) {
//		//	 encoderDrive(-0.5,0.5,0.5,-0.5, 750);
//		//	 encoderDrive(-0.5,-0.5,-0.5,-0.5,1000);
//		// } else if (frontColor2.alpha() - frontColor.alpha() >= 40) {
//		//	 encoderDrive(0.5,-0.5,-0.5,0.5,780);
//		//	 encoderDrive(-0.5,-0.5,-0.5,-0.5,1000);
//		// } else {
//		//	 encoderDrive(-0.5,-0.5,-0.5,-0.5,1000);
//		// }
//
//		rotate.setPosition(0);
//		rotate2.setPosition(1);
//		sleep(1300);
//		lift.setPower(0.3);
//		lift2.setPower(0.3);
//		sleep(850);
//		lift.setPower(0);
//		lift2.setPower(0);
//		rotate.setPosition(0.5);
//		rotate2.setPosition(0.5);
//
//		clamp.setPosition(0);
//		clamp2.setPosition(1);
//
//		encoderDrive(0.3,0.3,0.3,0.3,2650, 0);
//
//		clamp.setPosition(1);
//		clamp2.setPosition(0);
//
//		sleep(500);
//
//		lift.setPower(-0.3);
//		lift2.setPower(-0.3);
//		sleep(400);
//		lift.setPower(0);
//		lift2.setPower(0);
//
//
//		encoderDrive(-0.5,-0.5,-0.5,-0.5,950,0);
//		//encoderDrive(-0.5,-0.5,-0.5,-0.5,800,0);
//
//		// while(Math.abs((angle + angle2) / 2) > 0.01)
//		//	 angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//		//	 angle2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//		//	 telemetry.addData("rotation", (angle + angle2) / 2);
//		//	 telemetry.update();
//		//	 if((angle + angle2) / 2 < 0) {
//		//		 move(0,0,0.02);
//		//	 }
//		//	 else if((angle + angle2) / 2 > 0) {
//		//		 move(0,0,-0.02);
//		//	 }
//		// }
//
//		move(0,-0.5,0);
//		while(opModeIsActive() && groundColor.blue() < 30) {
//			double angle3 = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//			double angle4 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//			leftFront.setPower(0.3 - angle3);
//			leftBack.setPower(-0.38 - angle3);
//			rightFront.setPower(-0.3 + angle3);
//			rightBack.setPower(0.38 + angle3);
//			idle();
//		}
//
//		encoderDrive(-0.5,0.5,0.5,-0.5,4400,0);
//
//		lift.setPower(-0.3);
//		lift2.setPower(-0.3);
//		sleep(800);
//		lift.setPower(0);
//		lift2.setPower(0);
//
//		encoderDrive(0.5,0.5,0.5,0.5,1000,0);
//
//		clamp.setPosition(0);
//		clamp2.setPosition(1);
//		sleep(800);
//
//		encoderDrive(-0.5,-0.5,-0.5,-0.5,800,0);
//
//		encoderDrive(-0.5,0.5,-0.5,0.5,3500);
//
//		pull.setPosition(0);
//
//		encoderDrive(-0.3,-0.3,-0.3,-0.3,2400);
//
//		pull.setPosition(1);
//		sleep(1000);
//
//		encoderDrive(0.3, 0.6, 0.3, 0.6, 4500);
//
//		pull.setPosition(0);
//		lift.setPower(0.3);
//		lift2.setPower(0.3);
//		sleep(800);
//		lift.setPower(0);
//		lift2.setPower(0);
//
//
//		encoderDrive(-1,-1,-1,-1,800, -Math.PI/2);
//
//		encoderDrive(-1,1,1,-1,1000, -Math.PI/2);
//
//		encoderDrive(1,1,1,1,1500, -Math.PI/2);
//	}
//
//		private void move(double y, double x, double turn) {
//			double speed = Math.hypot(y, x);
//			double robotAngle = -Math.atan2(y, x) - Math.PI / 4;
//
//			leftFront.setPower(speed * Math.sin(robotAngle) + turn);
//			rightFront.setPower(speed * Math.cos(robotAngle) - turn);
//			leftBack.setPower(speed * Math.cos(robotAngle) + turn);
//			rightBack.setPower(speed * Math.sin(robotAngle) - turn);
//		}
//		private void move(double angle, double power) {
//			leftFront.setPower(power * Math.sin(Math.toRadians(angle)));
//			rightFront.setPower(power * Math.cos(Math.toRadians(angle)));
//			leftBack.setPower(power * Math.cos(Math.toRadians(angle)));
//			rightBack.setPower(power * Math.sin(Math.toRadians(angle)));
//		}
//		private void encoderDrive(double power1, double power2, double power3, double power4, double distance, double runAngle) {
//			//RUN_TO_POSITION allows the robots motors to remain “busy” until they finish running to a certain point
//
//			leftFront.setTargetPosition((int)Math.round(distance*-power1) + leftFront.getCurrentPosition());
//			rightFront.setTargetPosition((int)Math.round(distance*-power2) + rightFront.getCurrentPosition());
//			leftBack.setTargetPosition((int)Math.round(distance*-power3) + leftBack.getCurrentPosition());
//			rightBack.setTargetPosition((int)Math.round(distance*-power4) + rightBack.getCurrentPosition());
//
//			leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//			leftFront.setPower(-power1);
//			rightFront.setPower(-power2);
//			leftBack.setPower(-power3);
//			rightBack.setPower(-power4);
//
//			while(opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
//				double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
//				leftFront.setPower(-power1 - (angle - runAngle) / Math.PI);
//				leftBack.setPower(-power3 - (angle - runAngle) / Math.PI);
//				rightFront.setPower(-power2 + (angle - runAngle) / Math.PI);
//				rightBack.setPower(-power4 + (angle - runAngle) / Math.PI);
//				idle();
//			}
//
//			//motors have reached their positions, set their power to zero
//			leftFront.setPower(0);
//			rightFront.setPower(0);
//			leftBack.setPower(0);
//			rightBack.setPower(0);
//
//			//run with encoder only uses the encoders to regulate the speed of the motors
//			leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//			rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//			leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//			rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	}
//	private void encoderDrive(double power1, double power2, double power3, double power4, double distance) {
//			//RUN_TO_POSITION allows the robots motors to remain “busy” until they finish running to a certain point
//
//			leftFront.setTargetPosition((int)Math.round(distance*-power1) + leftFront.getCurrentPosition());
//			rightFront.setTargetPosition((int)Math.round(distance*-power2) + rightFront.getCurrentPosition());
//			leftBack.setTargetPosition((int)Math.round(distance*-power3) + leftBack.getCurrentPosition());
//			rightBack.setTargetPosition((int)Math.round(distance*-power4) + rightBack.getCurrentPosition());
//
//			leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//			leftFront.setPower(-power1);
//			rightFront.setPower(-power2);
//			leftBack.setPower(-power3);
//			rightBack.setPower(-power4);
//
//			while(opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy());
//
//			//motors have reached their positions, set their power to zero
//			leftFront.setPower(0);
//			rightFront.setPower(0);
//			leftBack.setPower(0);
//			rightBack.setPower(0);
//
//			//run with encoder only uses the encoders to regulate the speed of the motors
//			leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//			rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//			leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//			rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	}
//}
