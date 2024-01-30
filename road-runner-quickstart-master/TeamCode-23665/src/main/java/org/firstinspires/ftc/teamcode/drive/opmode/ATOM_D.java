package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "ATOM_D")
public class ATOM_D extends LinearOpMode {

    private Servo _1;
    private Servo _2;
    private DcMotor RL;
    private DcMotor RR;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor slider;
    private Servo manita;
    private DcMotor intake;
    private Servo launcher;


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double sliderSpeed;
        double speed;
        int initialSliderPosition;
        double y;
        double x;
        double rx;
        double denominator;
        double manitaPosition;
        RL = hardwareMap.get(DcMotor.class, "RL");
        RR = hardwareMap.get(DcMotor.class, "RR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        slider = hardwareMap.get(DcMotor.class, "slider");
        manita = hardwareMap.get(Servo.class, "manita");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(Servo.class, "launcher");
        //

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // gripper_close = true;
            sliderSpeed = 1.0;
            speed = 0.6;
            manitaPosition = manita.getPosition();
            /*
            RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            */
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Reverse the right side motors
            /*
            RL.setDirection(DcMotorSimple.Direction.REVERSE);
            FL.setDirection(DcMotorSimple.Direction.REVERSE);
            */

//            manita.setDirection(Servo.Direction.REVERSE);
            initialSliderPosition = slider.getCurrentPosition();
            slider.setDirection(DcMotorSimple.Direction.REVERSE);
            slider.setTargetPosition(initialSliderPosition);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(sliderSpeed);
            manita.scaleRange(0.2, 0.70);
            while (opModeIsActive()) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speed,
                                -gamepad1.left_stick_x * speed,
                                -gamepad1.right_stick_x * speed
                        )
                );

                drive.update();

                if (gamepad1.right_bumper) {
                    manita.setPosition(0.25);
                }
                if (gamepad1.left_bumper) {
                    manita.setPosition(1.0);
                }
                if (gamepad1.dpad_right && manita.getPosition() < 0.85) {
                    manita.setPosition(manita.getPosition() - 0.1);
                    manitaPosition = manita.getPosition();

                }
                if (gamepad1.dpad_left && manita.getPosition() > 0.01) {
                    manita.setPosition(manita.getPosition() - 0.1);
                    manitaPosition = manita.getPosition();
                }
/*
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
                FR.setPower(frontRightPower);
                RR.setPower(backRightPower);
                FL.setPower(frontLeftPower);
                RL.setPower(backLeftPower);*/


                if (gamepad1.a) {
                    slider.setTargetPosition(initialSliderPosition + 1200);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.x) {
                    slider.setTargetPosition(initialSliderPosition + 2000);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.y) {
                    slider.setTargetPosition(initialSliderPosition + 3210);
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

                if(gamepad1.right_trigger>0.0){
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(0.75);

                }else if(gamepad1.left_trigger>0.0){
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.75);
                }else{
                    intake.setPower(0.0);
                }

                if(gamepad1.options){
                    launcher.setPosition(0.00);
                }else if(gamepad1.share) {
                    launcher.setPosition(1.00);
                }
                telemetry.addData("gripper", manita.getPosition());
                telemetry.addData("gripper ", manita.getPosition());
                telemetry.addData("slider position", slider.getCurrentPosition());
                telemetry.addData("starting Pos.", initialSliderPosition);
                telemetry.addData("Manita Pos.", manita.getPosition());
                telemetry.update();
            }
        }
    }
}