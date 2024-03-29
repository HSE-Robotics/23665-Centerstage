/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.lang.String;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Pose2d;


import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Blue Back Stage", group = "Concept")

public class BlueBackstageATOM extends LinearOpMode {
    private DcMotor RR;
    private DcMotor FR;
    private DcMotor FL;
    private DcMotor RL;
    private DcMotor slider;
    private DcMotor intake;
    private Servo manita;
    private Servo launcher;
    int Initial_Position;
    ElapsedTime timelapse;
    double ticks_per_revolution;
    double wheel_circumference;
    double ticks_per_inch;
    int ticks_to_destination;
    int distance_to_travel;
    double speed;
    int initialPosition;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/BlueCone.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "azul cone",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    //private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        //initTfod();

        int timeBack;


        RR = hardwareMap.get(DcMotor.class, "RR");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        RL = hardwareMap.get(DcMotor.class, "RL");
        slider = hardwareMap.get(DcMotor.class, "slider");
        manita = hardwareMap.get(Servo.class, "manita");
        launcher = hardwareMap.get(Servo.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        wheel_circumference = 11.775;
        ticks_per_revolution = 528;
        ticks_per_inch = ticks_per_revolution / wheel_circumference;
        distance_to_travel = 60;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);

        if (opModeIsActive() && !isStopRequested()) {
            while (opModeIsActive()) {


                initialPosition = slider.getCurrentPosition();

                slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slider.setDirection(DcMotor.Direction.REVERSE);


                //telemetryTfod();
                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
        //        if (gamepad1.dpad_down) {
          //          visionPortal.stopStreaming();
            //    } else if (gamepad1.dpad_up) {
              //      visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
       // visionPortal.close();

  //  }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
       // visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        // tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        wheel_circumference = 11.775;
        ticks_per_revolution = 528;
        ticks_per_inch = ticks_per_revolution / wheel_circumference;
        distance_to_travel = 60;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        speed = 0.25;
        boolean objDetected = false;
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            String cubePosition = "";
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            double startingX, startingY, stackX, stackY;
            double initHeading = Math.PI * 1.5;

            startingX = 16;
            startingY = 64.00;

            Pose2d startingPose = new Pose2d(startingX, startingY, initHeading);
            drive.setPoseEstimate(startingPose);


            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("- PositionCube", "%s", cubePosition);

            TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d(10.06, 62.47, Math.toRadians(270.00)))
                    .lineTo(new Vector2d(55.06, 62.50))
                    .build();


            if((x > 200 && x < 310) && (y>100 && y < 220)){
                int initPos;
                objDetected = true;
                initPos = slider.getCurrentPosition();
                cubePosition = "Center";
                slider.setTargetPosition(initialPosition + 1163);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(0.7);

                telemetry.addData("Target Position:", "%s", initialPosition + 150);
                telemetry.update();
                sleep(150);

               TrajectorySequence untitled2 = drive.trajectorySequenceBuilder(new Pose2d(10.06, 62.47, Math.toRadians(270.00)))
                        .lineTo(new Vector2d(55.06, 62.50))
                        .lineTo(new Vector2d(12.06, 45.00))
                        .lineToLinearHeading(new Pose2d(58.00, 25.00, Math.toRadians(0.00)))
                        .addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()
                            manita.setPosition(0.65);
                            sleep(150);
                            // Run your action in here!
                        })
                        .lineToLinearHeading(new Pose2d(69.50, 26.00, Math.toRadians(270.00)))
                        .addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()

                            // Run your action in here!
                            sleep(500);

                            launcher.setPosition(1);
                            sleep(150);
                        })
                        .lineTo(new Vector2d(36.47,25.00))
                        .lineToLinearHeading(new Pose2d(36.47, 48.75, Math.toRadians(115.00)))
                        .lineToLinearHeading(new Pose2d(63.46, 63.06, Math.toRadians(180.00)))
                        .build();
                drive.setPoseEstimate(untitled1.start());
                drive.followTrajectorySequence(untitled1);
                armDown(initialPosition);
/*
                RightServo.setPower(1.0);
                LeftServo.setPower(-1.0);
                sleep(2500);
                ArmServo.setPosition(0.75);
                RightServo.setPower(0.0);
                LeftServo.setPower(0.0);

                TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d(54.38, 30.00, Math.toRadians(0.00)))
                        .splineTo(new Vector2d(48.60, 59.15), Math.toRadians(92.94))
                        .splineToLinearHeading(new Pose2d(70.00, 63.37, Math.toRadians(180.00)), Math.toRadians(67.91))
                        .build();*/
                drive.setPoseEstimate(untitled2.start());
                drive.followTrajectorySequence(untitled2);

                TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startingPose)
                        .lineTo(new Vector2d(16,30))
                        .lineTo(new Vector2d(16, 25))
                        .build();
                drive.followTrajectorySequence(seq1);


                break;

            }else if((x > 0 && x < 120) && (y>200 && y < 300)){
                cubePosition = "Left";
                int initPos;
                objDetected = true;
                initPos = slider.getCurrentPosition();
                cubePosition = "Left";
                slider.setTargetPosition(initialPosition + 1163);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(0.7);

                telemetry.addData("Target Position:", "%s", initialPosition + 150);
                telemetry.update();
                sleep(150);

                TrajectorySequence untitled2 = drive.trajectorySequenceBuilder(new Pose2d(10.06, 62.47, Math.toRadians(270.00)))
                        .lineTo(new Vector2d(55.06, 62.50))
                        /*.lineTo(new Vector2d(12.06, 45.00))
                        .lineToLinearHeading(new Pose2d(58.00, 25.00, Math.toRadians(0.00)))
                        .addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()
                            manita.setPosition(0.65);
                            sleep(150);
                            // Run your action in here!
                        })
                        .lineToLinearHeading(new Pose2d(69.50, 26.00, Math.toRadians(270.00)))
                        .addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()

                            // Run your action in here!
                            sleep(500);

                            launcher.setPosition(1);
                            sleep(150);
                        })
                        .lineTo(new Vector2d(36.47,25.00))
                        .lineToLinearHeading(new Pose2d(36.47, 48.75, Math.toRadians(115.00)))
                        .lineToLinearHeading(new Pose2d(63.46, 63.06, Math.toRadians(180.00)))*/
                        .build();
                drive.setPoseEstimate(untitled2.start());
                drive.followTrajectorySequence(untitled2);
                armDown(initialPosition);
/*
                RightServo.setPower(1.0);
                LeftServo.setPower(-1.0);
                sleep(2500);
                ArmServo.setPosition(0.75);
                RightServo.setPower(0.0);
                LeftServo.setPower(0.0);

                TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d(54.38, 30.00, Math.toRadians(0.00)))
                        .splineTo(new Vector2d(48.60, 59.15), Math.toRadians(92.94))
                        .splineToLinearHeading(new Pose2d(70.00, 63.37, Math.toRadians(180.00)), Math.toRadians(67.91))
                        .build();*/
                drive.setPoseEstimate(untitled1.start());
                drive.followTrajectorySequence(untitled1);

                TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startingPose)
                        //.lineTo(new Vector2d(16,30))
                        // .lineTo(new Vector2d(16, 25))
                        .build();
                drive.followTrajectorySequence(seq1);


                break;
            }else if((x > 480 && x < 550) && (y>200 && y < 300)){
                cubePosition = "Right";
                int initPos;
                objDetected = true;
                initPos = slider.getCurrentPosition();
                cubePosition = "Right";
                slider.setTargetPosition(initialPosition + 1163);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(0.7);

                telemetry.addData("Target Position:", "%s", initialPosition + 150);
                telemetry.update();
                sleep(150);

                TrajectorySequence untitled3 = drive.trajectorySequenceBuilder(new Pose2d(10.06, 62.47, Math.toRadians(270.00)))
                        .lineTo(new Vector2d(55.06, 62.50))
                        /*.lineTo(new Vector2d(12.06, 45.00))
                        .lineToLinearHeading(new Pose2d(58.00, 25.00, Math.toRadians(0.00)))
                        .addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()
                            manita.setPosition(0.65);
                            sleep(150);
                            // Run your action in here!
                        })
                        .lineToLinearHeading(new Pose2d(69.50, 26.00, Math.toRadians(270.00)))
                        .addDisplacementMarker(() -> {
                            // This marker runs after the first splineTo()

                            // Run your action in here!
                            sleep(500);

                            .setPosition(1);
                            sleep(150);
                        })
                        .lineTo(new Vector2d(36.47,25.00))
                        .lineToLinearHeading(new Pose2d(36.47, 48.75, Math.toRadians(115.00)))
                        .lineToLinearHeading(new Pose2d(63.46, 63.06, Math.toRadians(180.00)))*/
                        .build();
                drive.setPoseEstimate(untitled1.start());
                drive.followTrajectorySequence(untitled1);
                armDown(initialPosition);
/*
                RightServo.setPower(1.0);
                LeftServo.setPower(-1.0);
                sleep(2500);
                ArmServo.setPosition(0.75);
                RightServo.setPower(0.0);
                LeftServo.setPower(0.0);

                TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d(54.38, 30.00, Math.toRadians(0.00)))
                        .splineTo(new Vector2d(48.60, 59.15), Math.toRadians(92.94))
                        .splineToLinearHeading(new Pose2d(70.00, 63.37, Math.toRadians(180.00)), Math.toRadians(67.91))
                        .build();*/
                drive.setPoseEstimate(untitled1.start());
                drive.followTrajectorySequence(untitled1);

                TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startingPose)
                        //.lineTo(new Vector2d(16,30))
                        // .lineTo(new Vector2d(16, 25))
                        .build();
                drive.followTrajectorySequence(seq1);


                break;
            }




        }   // end for() loop

    }   // end method telemetryTfod()

    private void stopR(){
        RR.setPower(0.0);
        FR.setPower(0.0);
        RL.setPower(0.0);
        FL.setPower(0.0);
    }
    private void stopArm(){
       slider .setPower(0.0);
    }
    private void armToScore(){
        slider.setTargetPosition(slider.getCurrentPosition() + 600);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
    }

    private void armDown(int initialPosition){
        slider.setTargetPosition(initialPosition);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
        while (slider.isBusy()) {

        }
        slider.setPower(0.0);
    }
    /**
     * Drive Backwards / Forward
     */
    private void drive(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        RL.setTargetPosition(RL.getCurrentPosition() + ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition()+ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition()+ticks_to_destination);
        RR.setTargetPosition(RR.getCurrentPosition()+ticks_to_destination);
        FR.setPower(Power);
        RR.setPower(Power);
        RL.setPower(Power);
        FL.setPower(Power);
        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (RL.isBusy() && RR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {
            //telemetry.addData("BL - ", BL.getCurrentPosition() + " - " + BL.getTargetPosition());
            //  telemetry.addData("FL", "%.0i - %.0i", FL.getCurrentPosition(), FL.getTargetPosition());
            // telemetry.addData("FR", "%.0i - %.0i", FR.getCurrentPosition(), FR.getTargetPosition());
            // telemetry.addData("BR", "%.0i - %.0i", BR.getCurrentPosition(), BR.getTargetPosition());
            // telemetry.update();
        }
    }
    private void driveBack(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        RL.setTargetPosition(RL.getCurrentPosition() - ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition()-ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition()-ticks_to_destination);
        RR.setTargetPosition(RR.getCurrentPosition()-ticks_to_destination);
        FR.setPower(Power);
        RR.setPower(Power);
        RL.setPower(Power);
        FL.setPower(Power);
        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (RL.isBusy() && RR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {
            //telemetry.addData("BL", "%.0f - %.0f", BL.getCurrentPosition(), BL.getTargetPosition());
            //telemetry.addData("FL", "%.0f - %.0f", FL.getCurrentPosition(), FL.getTargetPosition());
            //telemetry.addData("FR", "%.0f - %.0f", FR.getCurrentPosition(), FR.getTargetPosition());
            //telemetry.addData("BR", "%.0f - %.0f", BR.getCurrentPosition(), BR.getTargetPosition());

        }
    }
    private void rotateLeft(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        RL.setTargetPosition(RL.getCurrentPosition() - ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition()-ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition()+ticks_to_destination);
        RR.setTargetPosition(RR.getCurrentPosition()+ticks_to_destination);
        FR.setPower(Power);
        RR.setPower(Power);
        RL.setPower(Power);
        FL.setPower(Power);
        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (RL.isBusy() && RR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {

        }
    }
    private void rotateRight(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        RL.setTargetPosition(RL.getCurrentPosition() + ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition()+ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition()-ticks_to_destination);
        RR.setTargetPosition(RR.getCurrentPosition()-ticks_to_destination);
        FR.setPower(Power);
        RR.setPower(Power);
        RL.setPower(Power);
        FL.setPower(Power);
        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (RL.isBusy() && RR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {

        }
    }
    /**
     * Strafe Right
     */
    private void strafe_left(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        RL.setTargetPosition(RL.getCurrentPosition() + ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition() - ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition() + ticks_to_destination);
        RR.setTargetPosition(RR.getCurrentPosition() - ticks_to_destination);
        FR.setPower(Power);
        RR.setPower(Power);
        RL.setPower(Power);
        FL.setPower(Power);
        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (RL.isBusy() && RR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {

        }
    }
    private void strafe_right(double Power, double distance_to_travel, double ticks_per_inch) {
        int ticks_to_destination;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        RL.setTargetPosition(RL.getCurrentPosition() - ticks_to_destination);
        FL.setTargetPosition(FL.getCurrentPosition() + ticks_to_destination);
        FR.setTargetPosition(FR.getCurrentPosition() - ticks_to_destination);
        RR.setTargetPosition(RR.getCurrentPosition() + ticks_to_destination);
        FR.setPower(Power);
        RR.setPower(Power);
        RL.setPower(Power);
        FL.setPower(Power);
        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (RL.isBusy() && RR.isBusy() &&
                FL.isBusy() && FR.isBusy()) {

        }
    }
}   // end class
