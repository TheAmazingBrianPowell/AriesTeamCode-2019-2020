package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BestAutonomous extends Bendy {

    @Override
    public void runOpMode() {
        int skyPosition = 0;
        //sets up the robot
        setUp();
        gyroCalibrate();
        vuforiaInit();
        telemetry.addData("status", "ready");
        telemetry.update();

        waitForStart();

        drive(0,-0.75,0,400);
        encoderStop();
        //gyroAlign(0);
        drive(0.75,0,0,1400);
        encoderStop();
        sleep(500);
        if(!vuforiaSkystone()) {
            drive(0,0.75,0,600);
            sleep(500);
            skyPosition = 1;
            if(!vuforiaSkystone()) {
                drive(0,0.75,0,600);
                skyPosition = 2;
            }
        }
        encoderStop();

        //gyroAlign(0);

        drive(-0.75,0,0,400);
        setPosition("rotate", 0);
        setPosition("rotate2", 1);
        sleep(200);
        clamp();
        sleep(1800);
        setPosition("rotate", 0.5);
        setPosition("rotate2", 0.5);
        encoderStop();

        //gyroAlign(0);

        drive(0.75,0,0,700);
        encoderStop();
        drive(0.75,0,0,300);
        clamp();
        encoderStop();


        drive(-0.75,0,0,700);
        encoderStop();
        //move("lift", 1, 100);
        //move("lift2", 1, 100);
        //encoderStop("lift");
        //gyroAlign(0);
        drive(0,-0.5,0);

        while(groundColor.blue() < 35 && opModeIsActive()) idle();
        gyroAlign(0);
        drive(0,0.75,0,3000);
        encoderStop();

        move("lift2", -1, 100);
        move("lift", 1, 100);
        encoderStop("lift");
        encoderStop("lift2");

        drive(0.75,0,0,300);

        clamp();

        gyroAlign(180);
    }
}
