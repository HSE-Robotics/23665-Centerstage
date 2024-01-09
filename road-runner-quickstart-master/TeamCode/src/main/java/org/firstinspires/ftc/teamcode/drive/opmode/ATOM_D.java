package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "ATOM_D")
public class ATOM_D extends LinearOpMode {

    private Servo _1;
    private Servo _2;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor slider;
    private Servo manita;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double sliderSpeed;
        double speed;
        int initialSliderPosition;
        double y;
        double x;
        double rx;
        double denominator;

        leftRear = hardwareMap.get(DcMotor.class, "RL");
        rightRear = hardwareMap.get(DcMotor.class, "RR");
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");

        slider = hardwareMap.get(DcMotor.class, "slider");
        manita = hardwareMap.get(Servo.class, "manita");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // gripper_close = true;
            sliderSpeed = 1.0;
            speed = 0.6;

            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Reverse the right side motors

            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

            manita.setDirection(Servo.Direction.REVERSE);
            initialSliderPosition = slider.getCurrentPosition();
            slider.setDirection(DcMotorSimple.Direction.FORWARD);
            slider.setTargetPosition(initialSliderPosition);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(sliderSpeed);
            manita.scaleRange(0.2, 1.0);
            while (opModeIsActive()) {
                if (gamepad1.right_bumper) {
                    manita.setPosition(1.0);
                }
                if (gamepad1.left_bumper) {
                    manita.setPosition(0.0);
                }

                // Remember, this is reversed!
                y = -gamepad1.left_stick_y * speed;
                x = gamepad1.left_stick_x * speed * 1.1;
                // Counteract imperfect strafing
                rx = gamepad1.right_stick_x * speed;

                // Denominator is the largest motor power
                // (absolute value) or 1
                // This ensures all the powers mantain
                // the same ratio, but only when at least one is
                // out of the range [-
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;
                // Make sure your ID's match your configuration
                rightFront.setPower(frontRightPower);
                rightRear.setPower(backRightPower);
                leftFront.setPower(frontLeftPower);
                leftRear.setPower(backLeftPower);


                if (gamepad1.a) {
                    slider.setTargetPosition(initialSliderPosition + 1200);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.x) {
                    slider.setTargetPosition(initialSliderPosition + 2000);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.y) {
                    slider.setTargetPosition(initialSliderPosition + 2750);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.b) {
                    slider.setTargetPosition(initialSliderPosition);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                }
                if (gamepad1.start) {
                    initialSliderPosition = slider.getCurrentPosition();
                }
                if (gamepad1.dpad_up) {
                    slider.setTargetPosition(slider.getCurrentPosition() + 100);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                }
                if (gamepad1.dpad_down) {
                    slider.setTargetPosition(slider.getCurrentPosition() - 100);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                }
                if (gamepad1.dpad_right) {
                    manita.setPosition(1.0);
                    slider.setTargetPosition(slider.getCurrentPosition() + 50);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);

                }
                telemetry.addData("gripper", manita.getPosition());
                telemetry.addData("gripper ", manita.getPosition());
                telemetry.addData("slider position",slider.getCurrentPosition());
                telemetry.addData("starting Pos.", initialSliderPosition);
                telemetry.update();
            }
        }
    }
}