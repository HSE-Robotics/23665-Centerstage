package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "ATOM_D_CentricField")
public class ATOM_D_CentricField extends LinearOpMode {

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
        int initialPotition;
        double y;
        double x;
        double rx;
        //double denominator;

        //_1 = hardwareMap.get(Servo.class, "1");
        //_2 = hardwareMap.get(Servo.class, "2");

        leftRear = hardwareMap.get(DcMotor.class, "RL");
        rightRear = hardwareMap.get(DcMotor.class, "RR");
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");

        slider = hardwareMap.get(DcMotor.class, "slider");
        manita = hardwareMap.get(Servo.class, "manita");
        IMU imu;

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // gripper_close = true;
            sliderSpeed = 0.75;
            speed = 0.6;

            //_1.setDirection(Servo.Direction.REVERSE);
            //_2.setDirection(Servo.Direction.FORWARD);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Reverse the right side motors

            rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

            manita.setDirection(Servo.Direction.REVERSE);
            initialPotition = slider.getCurrentPosition() + 85;
            slider.setDirection(DcMotorSimple.Direction.FORWARD);
            slider.setTargetPosition(initialPotition);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(sliderSpeed);
            manita.scaleRange(0.5, 1);
            while (opModeIsActive()) {
                if (gamepad1.right_bumper) {
                    manita.setPosition(0.99);
                    //_1.setPosition(1);
                    //_2.setPosition(1);
                }
                if (gamepad1.left_bumper) {
                    manita.setPosition(0);
                    // _1.setPosition(0);
                    //_2.setPosition(0);
                }

                // Remember, this is reversed!
                y = -gamepad1.left_stick_y * speed;
                x = gamepad1.left_stick_x * speed * 1.1;
                // Counteract imperfect strafing
                rx = -gamepad1.right_stick_x * speed;

                // Denominator is the largest motor power
                // (absolute value) or 1
                // This ensures all the powers mantain
                // the same ratio, but only when at least one is
                // out of the range [-
                // denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                /*
                //denominator = JavaUtil.averageOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;
                */

                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;


                // Make sure your ID's match your configuration
                rightFront.setPower(frontRightPower);
                rightRear.setPower(backRightPower);
                leftFront.setPower(frontLeftPower);
                leftRear.setPower(backLeftPower);


                if (gamepad1.a) {
                    slider.setTargetPosition(initialPotition + 1850);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.x) {
                    slider.setTargetPosition(initialPotition + 2900);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.y) {
                    slider.setTargetPosition(initialPotition + 4000);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.b) {
                    slider.setTargetPosition(initialPotition);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                }
                if (gamepad1.start) {
                    initialPotition = slider.getCurrentPosition();
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
                telemetry.addData("gripper", manita.getPosition());
                telemetry.addData("gripper ", manita.getPosition());
                telemetry.addData("slider position", slider.getCurrentPosition());
                telemetry.addData("starting Pos.", initialPotition);
                telemetry.update();
            }
        }
    }
}