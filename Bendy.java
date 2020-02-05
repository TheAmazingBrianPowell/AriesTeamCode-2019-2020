package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class Bendy {
	private DcMotor[] motors = new DcMotor[7];
	private double[] velocity = {0,0,0,0,0,0,0};
	private String[] names = {"a", "b", "x", "y"};
	private boolean[] prevButtons = {false, false, false, false};
	Bendy(HardwareMap hardwareMap) {
		motors[0] = hardwareMap.get(DcMotor.class, "leftFront");
		motors[1] = hardwareMap.get(DcMotor.class, "rightFront");
		motors[2] = hardwareMap.get(DcMotor.class, "rightBack");
		motors[3] = hardwareMap.get(DcMotor.class, "leftBack");
		motors[4] = hardwareMap.get(DcMotor.class, "lift");
		motors[5] = hardwareMap.get(DcMotor.class, "lift2");
		motors[6] = hardwareMap.get(DcMotor.class, "slide");
		
		for(int i = 0; i < motors.length; i++) {
			motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			if(i == 1 || i == 2) motors[i].setDirection(DcMotor.Direction.REVERSE);
			motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
		
		motors[0].setTargetPosition((int)(position * constrain(Math.sin(robotAngle) - turn, -1, 1)) + motors[0].getCurrentPosition());
		motors[1].setTargetPosition((int)(position * constrain(Math.cos(robotAngle) + turn, -1, 1)) + motors[1].getCurrentPosition());
		motors[2].setTargetPosition((int)(position * constrain(Math.sin(robotAngle) + turn, -1, 1)) + motors[2].getCurrentPosition());
		motors[3].setTargetPosition((int)(position * constrain(Math.cos(robotAngle) - turn, -1, 1)) + motors[3].getCurrentPosition());
		for(int i = 0; i < 4; i++) {
			motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
		drive(y,x,turn);
		
		while((motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy()));
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
}
