package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class BestAutonomous extends Bendy {
	int skyPosition = 0;
	
	
	
	@Override
	public void runOpMode() {
			//sets up the robot
			setUp(); 
			telemetry.addData("status", "callibrating gyrosensor");
			telemetry.update();
			gyroCalibrate();
			telemetry.addData("status", "initiallizing vuforia");
			telemetry.update();
			vuforiaInit();
			telemetry.addData("status", "ready");
			telemetry.update();
		
			waitForStart();
			
			drive(0,-1,0,700);
			gyroAlign(0);
			drive(1,0,0,1400);
			sleep(500);
			if(!vuforiaSkystone()) {
				drive(0,1,0,500);
				skyPosition = 1;
				sleep(500);
				if(!vuforiaSkystone()) {
					drive(0,1,0,500);
					skyPosition = 2;
				}
			}
			
			gyroAlign(0);
			
			drive(-1,0,0,400);
			setPosition("rotate", 0);
			setPosition("rotate2", 1);
			sleep(200);
			clamp();
			move("lift2", -1, 50);
			move("lift", 1, 50);
			encoderStop("lift");
			encoderStop("lift2");
			sleep(1800);
			setPosition("rotate", 0.5);
			setPosition("rotate2", 0.5);
			
			
			
			gyroAlign(0);
			
			drive(1,0,0,700);
			clamp();
			drive(1,0,0,300);
			
			
			drive(-1,0,0,700);
			//move("lift", 1, 100);
			//move("lift2", 1, 100);
			//encoderStop("lift");
			//gyroAlign(0);
			drive(0,0.5,0);
			
			while(groundColor.blue() < 35 && opModeIsActive()) idle();
			gyroAlign(0);
			drive(0,-1,0,3000);
			
			move("lift2", -1, 100);
			move("lift", 1, 100);
			encoderStop("lift");
			encoderStop("lift2");
			
			drive(1,0,0,300);
			
			clamp();
			
			gyroAlign(180);
	}
}
