package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//Tensor Flow and Vuforia import statements
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous
public class SmartAutonomous extends LinearOpMode  {
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private ElapsedTime currentTime = new ElapsedTime();
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private String stage = "Init";

    @Override
    public void runOpMode() {
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

        initVuforia();
        initTfod();
        telemetry.addData("Stage", stage);
        telemetry.update();
        waitForStart();
        stage = "find";
        currentTime.reset();
        if (opModeIsActive()) {
            while(stage.equals("find") && opModeIsActive()) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData("label", recognition.getLabel());
                        telemetry.addData("left, top, right, bottom", "%.03f , %.03f , %.03f , %.03f", recognition.getLeft(), recognition.getTop(), recognition.getRight(), recognition.getBottom());
                        telemetry.addData("Go",((recognition.getLeft() > 0) ? "Move Right" : "Move Left") + ((recognition.getRight() - recognition.getLeft() < 900) ? "Move Forward" : "Move Backwards") + (recognition.getRight() - recognition.getLeft()));
                        if(recognition.getLeft() > 0) {
                            move(0,-1,0);
                        }else{
                            move(0,1,0);
                        }
                        if(recognition.getRight() - recognition.getLeft() < 900) {
                            move(1,0,0);
                        }else{
                            move(-1,0,0);
                        }
                    }
                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }


    //Functions
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AbxWk0X/////AAABmbTHsmdPK0rWsHtl3bN7AfNWxaadLD6LGw0yvaK6Jk8EVl3jSPKMV/fht+xuCmYwDfcTu4T7KVtMRmLI8fezTp2sgVQ4J3/7GYGp/duLM3448Ir4ER1r4IoTPhdWXFRUS0V3F2TAgM4PT7KUd15dMOm6LqVwsOu1Msfgv7tjQuvl2Dc6k16VOE/IVcd9UK31Q9zx15cF3kVn5/y7PS+kKcOZPSUn8ghxiPVrU7x4/9QYx9HInyQ4b6rK+8+dPUkcPF2n+1EfrjXSkmCCQna2AZiTxv3ASgCOULQtYgjdGRetha6CJYOgZrT1xF+qUX+KM1s/QkyBU1tq3TC2JN6m/+zBMU+LeSx+ivhstDHnd5iP";
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset("Skystone.tflite", "Stone", "Skystone");
        if (tfod != null) {
            tfod.activate();
        }
    }
    private void encoderDrive(double power1, double power2, double power3, double power4, double distance) {
        //RUN_TO_POSITION allows the robots motors to remain “busy” until they finish running to a certain point
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition((int)Math.round(distance*power1) + leftFront.getCurrentPosition());
        rightFront.setTargetPosition((int)Math.round(distance*power2) + rightFront.getCurrentPosition());
        leftBack.setTargetPosition((int)Math.round(distance*power3) + leftBack.getCurrentPosition());
        rightBack.setTargetPosition((int)Math.round(distance*power4) + rightBack.getCurrentPosition());

        leftFront.setPower(power1);
        rightFront.setPower(power2);
        leftBack.setPower(power3);
        rightBack.setPower(power4);

        while(leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
            //Wait for motors to reach their positions...
        }

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

    //simulates joystick driving, as a function for easy movement
    private void move(double y, double x, double turn) {
        double radius = Math.hypot(x, y);

        //Uses arc tangent (inverse tangent) to find how we need to rotate the power of the robot in order to move in the direction we want it to
        double robotAngle = Math.atan2(x, y) - Math.PI / 4;

        //Note: Math.pow(value, 2) may be a method for achieving logarithmic controls, but negative numbers will become positive!

        //multiply the direction we need to travel in by the speed to create the power to each wheel, and also the amount that we are turning to get the power of the robot’s wheels
        leftFront.setPower(radius * Math.cos(robotAngle) + turn);
        rightFront.setPower(radius * Math.sin(robotAngle) - turn);
        leftBack.setPower(radius * Math.sin(robotAngle) + turn);
        rightBack.setPower(radius * Math.cos(robotAngle) - turn);
    }
}
