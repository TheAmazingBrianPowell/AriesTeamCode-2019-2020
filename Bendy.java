package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class Bendy {
	private DcMotor[] motors = new DcMotor[7];
	private Gamepad gamepad3;
	private Gamepad gamepad4;
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
	
	public void drive(double angle, double power) {
		motors[0].setPower(power * Math.sin(Math.toRadians(angle)));
		motors[1].setPower(power * Math.cos(Math.toRadians(angle)));
		motors[2].setPower(power * Math.sin(Math.toRadians(angle)));
		motors[3].setPower(power * Math.cos(Math.toRadians(angle)));
	}
	
	public void drive(double y, double x, double turn) {
		double speed = Math.hypot(y, x);
		double robotAngle = Math.atan2(y, x) - Math.PI / 4;

		motors[0].setPower(speed * Math.sin(robotAngle) - turn);
		motors[1].setPower(speed * Math.cos(robotAngle) + turn);
		motors[2].setPower(speed * Math.sin(robotAngle) + turn);
		motors[3].setPower(speed * Math.cos(robotAngle) - turn);
	}
	
	public Gamepad getGamepad(int i) {
		return (i == 0) ? gamepad3 : gamepad4;
	}
	
	public boolean buttonToggle(boolean a, boolean b, Telemetry telemetry) {
		telemetry.addData("a",a);
		telemetry.addData("b", b);
		telemetry.update();
		return a && !b;
	}
	
	public void updateControllers(Gamepad gamepad1, Gamepad gamepad2) {
		gamepad3 = gamepad1;
		gamepad4 = gamepad2;
	}
}

class Toggle {
	private boolean currentValue;
	private boolean prevValue;
	private boolean isToggled;
	Toggle(boolean current) {
		currentValue = current;
		prevValue = current;
	}
	public void updateToggle(boolean newValue) {
		prevValue = currentValue;
		currentValue = newValue;
		if(prevValue != currentValue) {
			isToggled = !isToggled;
		}
	}
	public boolean getState() {
		return isToggled;
	}
}
