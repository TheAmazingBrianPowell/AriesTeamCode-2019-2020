package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
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
    private static DcMotor[] motors = new DcMotor[7];
    private static Servo[] servos = new Servo[5];
    //private double[] velocity = {0,0,0,0,0,0,0};
    private static String[] names = {"a", "b", "x", "y"};
    private static boolean[] prevButtons = {false, false, false, false};
    private static BNO055IMU imu;
    private static boolean clamped = false;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false ;

    private static final float mmPerInch = 25.4f;

    private static final float stoneZ = 2.00f * mmPerInch;
    private VuforiaLocalizer vuforia = null;

    private VuforiaTrackable skystone;
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
        }

        for(int i = 0; i < motors.length; i++) {
            if(!isStopRequested()) {
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    void vuforiaInit() {
        float phoneXRotate	= 0;
        float phoneYRotate;
        float phoneZRotate	= 0;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AbxWk0X/////AAABmbTHsmdPK0rWsHtl3bN7AfNWxaadLD6LGw0yvaK6Jk8EVl3jSPKMV/fht+xuCmYwDfcTu4T7KVtMRmLI8fezTp2sgVQ4J3/7GYGp/duLM3448Ir4ER1r4IoTPhdWXFRUS0V3F2TAgM4PT7KUd15dMOm6LqVwsOu1Msfgv7tjQuvl2Dc6k16VOE/IVcd9UK31Q9zx15cF3kVn5/y7PS+kKcOZPSUn8ghxiPVrU7x4/9QYx9HInyQ4b6rK+8+dPUkcPF2n+1EfrjXSkmCCQna2AZiTxv3ASgCOULQtYgjdGRetha6CJYOgZrT1xF+qUX+KM1s/QkyBU1tq3TC2JN6m/+zBMU+LeSx+ivhstDHnd5iP";
        parameters.cameraDirection = CAMERA_CHOICE;

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
    }

    boolean vuforiaSkystone() {
        if(!isStopRequested()) {
            return (((VuforiaTrackableDefaultListener)skystone.getListener()).isVisible());
        } else {
            return false;
        }
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

	void drive(double angle, double power) {
        if(!isStopRequested()) {
            motors[0].setPower(power * Math.sin(Math.toRadians(angle)));
            motors[1].setPower(power * Math.cos(Math.toRadians(angle)));
            motors[2].setPower(power * Math.sin(Math.toRadians(angle)));
            motors[3].setPower(power * Math.cos(Math.toRadians(angle)));
        }
    }

    void drive(double y, double x, double turn) {
        if(!isStopRequested()) {
            double speed = Math.hypot(y, x);
            double robotAngle = Math.atan2(y, x) - Math.PI / 4;

            // do {
            motors[0].setPower(speed * Math.sin(robotAngle) - turn);
            motors[1].setPower(speed * Math.cos(robotAngle) + turn);
            motors[2].setPower(speed * Math.sin(robotAngle) + turn);
            motors[3].setPower(speed * Math.cos(robotAngle) - turn);
        }
        // } while(Math.abs(constrain(velocity[0],-1,1) - constrain(speed * Math.sin(robotAngle) - turn,-1,1)) > 0.05 && !teleop);
    }

    void drive(double y, double x, double turn, int position) {
        if(!isStopRequested()) {
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
    }

    void encoderStop() {
        while(motors[0].isBusy() && motors[1].isBusy() && motors[2].isBusy() && motors[3].isBusy() && opModeIsActive()) idle();
        if(!opModeIsActive()) {
            for(int i = 0; i < 4; i++) {
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motors[i].setPower(0);
            }
        }
    }

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
            return button && ! prevButtons[index];
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
