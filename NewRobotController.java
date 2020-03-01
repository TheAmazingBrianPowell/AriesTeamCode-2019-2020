package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class NewRobotController extends Bendy {

    @Override
    public void runOpMode() {
        setUp();

        initTensorFlow();

        waitForStart();

        double speedAdjust = 0.8;
        boolean isUp = false;
        boolean isClamped = false;
        boolean isRotated = false;
        int motorTimer = 0;
        while (opModeIsActive()) {

            drive(gamepad1.left_stick_y * speedAdjust, gamepad1.left_stick_x * speedAdjust, gamepad1.right_stick_x * speedAdjust);
            if (toggleButton("a", gamepad1.a)) {
                motorTimer = 0;
                isUp = !isUp;
            }

            if (isUp && motorTimer < 5000) {
                motors[4].setPower(0.5);
                motors[5].setPower(-0.5);
                motorTimer++;
            } else if (!isUp && motorTimer < 5000) {
                motors[4].setPower(-0.5);
                motors[5].setPower(0.5);
                motorTimer++;
            } else {
                motors[4].setPower(0);
                motors[5].setPower(0);
            }

            if (gamepad1.left_trigger > 0.01) motors[6].setPower(gamepad1.left_trigger);
            else if (gamepad1.right_trigger > 0.01) motors[6].setPower(-gamepad1.right_trigger);
            else motors[6].setPower(0);

            if (toggleButton("b", gamepad1.b)) {
                servos[0].setPosition(isClamped ? 0 : 1);
                servos[1].setPosition(isClamped ? 1 : 0);
                isClamped = !isClamped;
            }

            servos[2].setPosition(isUp ? 0.3 : 1);
            servos[3].setPosition(isUp ? 0.7 : 0);
        }
    }
}