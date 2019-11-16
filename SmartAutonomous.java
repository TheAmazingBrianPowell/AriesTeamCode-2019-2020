package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous
public class BlueSmart extends LinearOpMode {
   private DcMotor leftBack, rightBack, leftFront, rightFront, slide, lift, lift2;

   private ElapsedTime runtime = new ElapsedTime();

   private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
   private static final boolean PHONE_IS_PORTRAIT = false ;
  
   private static final float mmPerInch = 25.4f;
   private static final float mmTargetHeight = (6) * mmPerInch;
  
   private static final float stoneZ = 2.00f * mmPerInch;
  
   private static final float bridgeZ = 6.42f * mmPerInch;
   private static final float bridgeY = 23 * mmPerInch;
   private static final float bridgeX = 5.18f * mmPerInch;
   private static final float bridgeRotY = 59;
   private static final float bridgeRotZ = 180;
  
   private static final float halfField = 72 * mmPerInch;
   private static final float quadField  = 36 * mmPerInch;
  
   private OpenGLMatrix lastLocation = null;
   private VuforiaLocalizer vuforia = null;
   private boolean targetVisible = false;
   private float phoneXRotate    = 0;
   private float phoneYRotate    = 0;
   private float phoneZRotate    = 0;

   @Override
   public void runOpMode() {
       Servo clamp, pull;
       ColorSensor frontColor, groundColor;
       DistanceSensor frontDistance;
       List<VuforiaTrackable> allTrackables = new ArrayList<>();
      
       if(!isStopRequested()) {
           int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
           VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

           // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

           parameters.vuforiaLicenseKey = "AbxWk0X/////AAABmbTHsmdPK0rWsHtl3bN7AfNWxaadLD6LGw0yvaK6Jk8EVl3jSPKMV/fht+xuCmYwDfcTu4T7KVtMRmLI8fezTp2sgVQ4J3/7GYGp/duLM3448Ir4ER1r4IoTPhdWXFRUS0V3F2TAgM4PT7KUd15dMOm6LqVwsOu1Msfgv7tjQuvl2Dc6k16VOE/IVcd9UK31Q9zx15cF3kVn5/y7PS+kKcOZPSUn8ghxiPVrU7x4/9QYx9HInyQ4b6rK+8+dPUkcPF2n+1EfrjXSkmCCQna2AZiTxv3ASgCOULQtYgjdGRetha6CJYOgZrT1xF+qUX+KM1s/QkyBU1tq3TC2JN6m/+zBMU+LeSx+ivhstDHnd5iP";
           parameters.cameraDirection = CAMERA_CHOICE;

           //  Instantiate the Vuforia engine
           vuforia = ClassFactory.getInstance().createVuforia(parameters);

           VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

           VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
           stoneTarget.setName("Stone Target");
           VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
           blueRearBridge.setName("Blue Rear Bridge");
           VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
           redRearBridge.setName("Red Rear Bridge");
           VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
           redFrontBridge.setName("Red Front Bridge");
           VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
           blueFrontBridge.setName("Blue Front Bridge");
           VuforiaTrackable red1 = targetsSkyStone.get(5);
           red1.setName("Red Perimeter 1");
           VuforiaTrackable red2 = targetsSkyStone.get(6);
           red2.setName("Red Perimeter 2");
           VuforiaTrackable front1 = targetsSkyStone.get(7);
           front1.setName("Front Perimeter 1");
           VuforiaTrackable front2 = targetsSkyStone.get(8);
           front2.setName("Front Perimeter 2");
           VuforiaTrackable blue1 = targetsSkyStone.get(9);
           blue1.setName("Blue Perimeter 1");
           VuforiaTrackable blue2 = targetsSkyStone.get(10);
           blue2.setName("Blue Perimeter 2");
           VuforiaTrackable rear1 = targetsSkyStone.get(11);
           rear1.setName("Rear Perimeter 1");
           VuforiaTrackable rear2 = targetsSkyStone.get(12);
           rear2.setName("Rear Perimeter 2");
           allTrackables.addAll(targetsSkyStone);
           stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, stoneZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
           blueFrontBridge.setLocation(OpenGLMatrix.translation(-bridgeX, bridgeY, bridgeZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
           blueRearBridge.setLocation(OpenGLMatrix.translation(-bridgeX, bridgeY, bridgeZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
           redFrontBridge.setLocation(OpenGLMatrix.translation(-bridgeX, -bridgeY, bridgeZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));
           redRearBridge.setLocation(OpenGLMatrix.translation(bridgeX, -bridgeY, bridgeZ).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));
           red1.setLocation(OpenGLMatrix.translation(quadField, -halfField, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
           red2.setLocation(OpenGLMatrix.translation(-quadField, -halfField, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
           front1.setLocation(OpenGLMatrix.translation(-halfField, -quadField, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
           front2.setLocation(OpenGLMatrix.translation(-halfField, quadField, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
           blue1.setLocation(OpenGLMatrix.translation(-quadField, halfField, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
           blue2.setLocation(OpenGLMatrix.translation(quadField, halfField, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
           rear1.setLocation(OpenGLMatrix.translation(halfField, quadField, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
           rear2.setLocation(OpenGLMatrix.translation(halfField, -quadField, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

           if (CAMERA_CHOICE == BACK) {
               phoneYRotate = -90;
           } else {
               phoneYRotate = 90;
           }

           if (PHONE_IS_PORTRAIT) {
               phoneXRotate = 90;
           }

           // Next, translate the camera lens to where it is on the robot.
           // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
           final float CAMERA_FORWARD_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
           final float CAMERA_VERTICAL_DISPLACEMENT = 3.0f * mmPerInch;   // eg: Camera is 3 Inches above ground
           final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

           OpenGLMatrix robotFromCamera = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

           //Let all the trackable listeners know where the phone is.
           for (VuforiaTrackable trackable : allTrackables) {
               ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
           }
           targetsSkyStone.activate();
       }
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

       //Servos for intake control
       clamp = hardwareMap.get(Servo.class, "rotate");
       pull = hardwareMap.get(Servo.class, "pull");

       //color/distance sensor on the front of robot for sensing stones' distances from the robot
       frontColor = hardwareMap.get(ColorSensor.class, "oof");
       frontDistance = hardwareMap.get(DistanceSensor.class, "oof");
       groundColor = hardwareMap.get(ColorSensor.class, "colorSensor");
       waitForStart();
       runtime.reset();

       if (opModeIsActive()) {
           telemetry.addData("constrain", constrain(-5,-1,1));
           telemetry.update();
           //release the clamp
           clamp.setPosition(0);
           
           encoderDrive(0.5,0,0,10000);

           //move until the robot is close to the blocks
           move(0.3,0,0);
           while(opModeIsActive() && (frontDistance.getDistance(DistanceUnit.CM) >= 40 || Double.isNaN(frontDistance.getDistance(DistanceUnit.CM))));
           move(0,0,0);

           //move backwards at 0.2 power for 1000 encoder units
           encoderDrive(0.2,0,0,1000);

           //move until the skystone is visible to the phone's camera
           move(0,0.1,0);
           while(opModeIsActive() && !targetVisible) {
               targetVisible = false;
               for (VuforiaTrackable trackable : allTrackables) {
                   if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                       targetVisible = true;
                       OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                       if (robotLocationTransform != null) {
                           lastLocation = robotLocationTransform;
                       }
                       break;
                   }
               }
           }
           move(0,0,0);
           

           //Drive forwards for 1000 encoder units at 0.2 power
           encoderDrive(0.2,0,0,1000);
          
           //clamp onto stone
           clamp.setPosition(1);

           //move backwards 2000 encoder units
           encoderDrive(-0.2,0,0,2000);
          
           //turn about 90 degrees
           encoderDrive(0,0,0.2, 5000);
          
           encoderDrive(0.3,0,0,10000);

           telemetry.addData("ground color", groundColor.red() + "," + groundColor.blue());
           telemetry.update();
           //   move(0,1,0);
           //   while(opModeIsActive() && (groundSensor.red() > 50 || groundSensor.blue() > 50));
           //   move(0,0,0);
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
   private double constrain(double value, double min, double max) {
       return ((value > min) ? ((value < max) ? value : max) : min);
   }
   private void encoderDrive(double y, double x, double turn, double distance) {
       double speed = Math.hypot(y, x);
       double robotAngle = -Math.atan2(y, x) - Math.PI / 4;
      
       leftFront.setTargetPosition((int)Math.round(distance * constrain(Math.sin(robotAngle) + turn, -1, 1)) + leftFront.getCurrentPosition());
       rightFront.setTargetPosition((int)Math.round(distance * constrain(Math.cos(robotAngle) - turn, -1, 1)) + rightFront.getCurrentPosition());
       leftBack.setTargetPosition((int)Math.round(distance * constrain(Math.cos(robotAngle) + turn, -1, 1)) + leftBack.getCurrentPosition());
       rightBack.setTargetPosition((int)Math.round(distance * constrain(Math.sin(robotAngle) - turn, -1, 1)) + rightBack.getCurrentPosition());

       move(y,x,turn);
      
       leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       // wait for motors to reach their positions...
       while(leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy() && opModeIsActive());

       // motors have reached their positions, set their power to zero
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
