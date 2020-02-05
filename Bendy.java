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
	
	private void setVelocity(int index, double newVelocity, double delay) {
		velocity[index] = Math.sinh(velocity[index]) / delay + newVelocity * (1 - Math.sinh(1) / delay);
		if(velocity[index] > 1) velocity[index] = 1;
		else if(velocity[index] < -1) velocity[index] = -1;
		motors[index].setPower(velocity[index]);
	}
	
	public void drive(double angle, double power) {
		setVelocity(0, power * Math.sin(Math.toRadians(angle)), 10);
		setVelocity(1, power * Math.cos(Math.toRadians(angle)), 10);
		setVelocity(2, power * Math.sin(Math.toRadians(angle)), 10);
		setVelocity(3, power * Math.cos(Math.toRadians(angle)), 10);
	}
	
	public void drive(double y, double x, double turn) {
		double speed = Math.hypot(y, x);
		double robotAngle = Math.atan2(y, x) - Math.PI / 4;
		
		while(Math.abs(speed * Math.sin(robotAngle - turn) < 0.05) {
			setVelocity(0, speed * Math.sin(robotAngle) - turn, 10);
			setVelocity(1, speed * Math.cos(robotAngle) + turn, 10);
			setVelocity(2, speed * Math.sin(robotAngle) + turn, 10);
			setVelocity(3, speed * Math.cos(robotAngle) - turn, 10);
		}
	}
	
	public boolean toggleButton(String name, boolean button) {
		int index = 0;
		for(int i = 0; i < names.length; i++) {
			if(names[i].equals(name)) {
				index = i;
			}
		}
		boolean prevValue = buttons[index];
		buttons[index] = button;
		return button && ! buttons[index];
	}
}
