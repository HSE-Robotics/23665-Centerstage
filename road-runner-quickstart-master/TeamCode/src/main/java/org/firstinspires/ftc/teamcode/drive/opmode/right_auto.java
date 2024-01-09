package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous(name = "right_auto", preselectTeleOp = "Atom_D")
public class right_auto extends LinearOpMode {

    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;
    private DcMotor slider;
    private Servo gripper;

    double speed;
    Recognition recognition;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int Initial_Position;
        ElapsedTime timelapse;
        boolean objDetected;
        int timeDown;
        List<Recognition> recognitions;
        int index;
        int timeBack;
        double ticks_per_revolution;
        double wheel_circumference;
        double ticks_per_inch;
        int ticks_to_destination;
        int distance_to_travel;

        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();
        slider = hardwareMap.get(DcMotor.class, "slider");
        gripper = hardwareMap.get(Servo.class, "manita");
        gripper.scaleRange(0.5, 1);
        // Sample TFOD Op Mode
        // Initialize Vuforia.
        vuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        // Set isModelTensorFlow2 to true if you used a TensorFlow 2 tool,
        // such as ftc-ml, to create the model. Set isModelQuantized to
        // true if the model is quantized. Models created with ftc-ml are
        // quantized. Set inputSize to the image size corresponding to
        // the model. If your model is based on SSD MobileNet v2 320x320,
        // the image size is 300 (srsly!). If your model is based on
        // SSD MobileNet V2 FPNLite 320x320, the image size is 320.
        // If your model is based on SSD MobileNet V1 FPN 640x640 or
        // SSD MobileNet V2 FPNLite 640x640, the image size is 640.
        tfod.useModelFromFile("/sdcard/FIRST/tflitemodels/MightyHawks.tflite", JavaUtil.createListWith("HP", "HAWK", "Smile"), true, true, 300);
        // Set min confidence threshold to 0.7
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.5, true, true);
        // Initialize TFOD before waitForStart.
        // Activate TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfod.activate();
        // Enable following block to zoom in on target.
        tfod.setZoom(1.5, 16 / 9);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Wait for start command from Driver Station.
        waitForStart();
        wheel_circumference = 11.775;
        ticks_per_revolution = 528;
        ticks_per_inch = ticks_per_revolution / wheel_circumference;
        distance_to_travel = 60;
        ticks_to_destination = (int) (distance_to_travel * ticks_per_inch);
        speed = 0.55;
        if (opModeIsActive() && !isStopRequested()) {
            // Put run blocks here.
            timelapse = new ElapsedTime();
            timelapse.reset();
            objDetected = false;
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Get Initial Slider Position
            int initialPosition = slider.getCurrentPosition();
            // Close the Gripper
            closeGripper();
            sleep(500);
            // Slider Motor Should be reversed to be able to
            // use positive numbers for Positions
            slider.setDirection(DcMotorSimple.Direction.FORWARD);
            moveSlider(slider.getCurrentPosition() + 100);
            timeDown = 2700;
            //sleep(100);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


            double initHeading = Math.PI / 2.0;
            double stackHeading = Math.toRadians(0.00);
            double mediumHeading = Math.PI * 1.75;
            int mediumJunctionPosition = initialPosition + 2000;
            int lowJunctionPosition = initialPosition + 1200;
            int highJunctionPosition = initialPosition + 2750;
            int stackTopCone = initialPosition + 500;
            double startingX, startingY, stackX, stackY;
            double firstLowJunctionX = 50.00;
            double firstLowJunctionY = -33.00;
            double firstLowJunctionXBack = 50.00;
            double firstLowJunctionYBack = -20.00;
            startingX = 36.00;
            startingY = -64.00;
            stackY = -15.00;
            stackX = 63.00;

            Pose2d startingPose = new Pose2d(startingX, startingY, initHeading);
            drive.setPoseEstimate(startingPose);
            telemetry.addData("Robot Heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("Robot X", drive.getPoseEstimate().getX());
            telemetry.addData("Robot Y", drive.getPoseEstimate().getY());
            telemetry.update();





            while (opModeIsActive() && !objDetected && Math.round(timelapse.seconds()) < 20) {
                // Put loop blocks here.
                // Get a list of recognitions from TFOD.
                recognitions = tfod.getRecognitions();
                // Check the length of the recognitions

                // If list is empty, inform the user. Otherwise, go
                // through list and display info for each recognition.
                if (JavaUtil.listLength(recognitions) == 0) {
                    telemetry.addData("TFOD", "No items detected.");
                    telemetry.addData("elapsed time", Math.round(timelapse.seconds()));
                }
                else {
                    index = 0;
                    objDetected=true;
                    // Iterate through list and call a function to
                    // display info for each recognized object.
                    for (Recognition recognition_item : recognitions) {
                        recognition = recognition_item;
                        timeBack = 1900;
                        // Display info.
                        displayInfo(index);
                        // Increment index.
                        index = index + 1;
                        if (recognition.getLabel().equals("HP")) {
                            // HP has been detected, Parking position 2
                            break;
                        }
                        else if (recognition.getLabel().equals("HAWK")) {
                            // HAWK has been detected, parking position 1
                            TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startingPose)
                                    .strafeTo(new Vector2d(firstLowJunctionX,startingY))
                                    .lineTo(new Vector2d(firstLowJunctionX, firstLowJunctionY))
                                    .waitSeconds(.35)
                                    .lineTo(new Vector2d(firstLowJunctionX, firstLowJunctionY - 10.00))
                                    .strafeTo(new Vector2d(64.00,firstLowJunctionY-7.00))
                                    .splineToLinearHeading(new Pose2d(stackX + 5.00, stackY, stackHeading), Math.toRadians(80.00))
                                    .lineTo(new Vector2d(stackX - 1.0, stackY))
                                    .lineTo(new Vector2d(stackX, stackY))
                                    .addTemporalMarker(1.0, () ->
                                    {
                                        moveSlider(lowJunctionPosition);
                                    })
                                    .addTemporalMarker(2.75, () ->
                                    {
                                        openGripper();
                                    })
                                    .addTemporalMarker(4.0, () ->
                                    {
                                        moveSlider(stackTopCone);
                                    })


                                    .build();

                            drive.followTrajectorySequence(seq1);
                            //Grab top cone from stack and place it in low junction.
                            Pose2d secondStartingPose = new Pose2d(stackX - 1.0, stackY, stackHeading);
                            TrajectorySequence seq2 = drive.trajectorySequenceBuilder(secondStartingPose)
                                    .waitSeconds(0.15)
                                    .lineTo(new Vector2d(stackX+2.0,stackY))
                                    .lineTo(new Vector2d(45.00,stackY))
                                  //  .lineToLinearHeading(new Pose2d(-50.00, -20.00, Math.toRadians(270)))
                                    .lineToLinearHeading(new Pose2d(55.00, stackY, Math.toRadians(270)))
                                    .lineTo(new Vector2d(firstLowJunctionXBack, firstLowJunctionYBack))
                                    .addTemporalMarker(0.0, () ->
                                    {
                                        closeGripper();
                                    })
                                    .addTemporalMarker(0.15, () ->
                                    {
                                        moveSlider(lowJunctionPosition);
                                    })
                                    .addTemporalMarker(1.5, () ->
                                    {
                                        openGripper();
                                    })
                                    .build();
                            drive.followTrajectorySequence(seq2);

                            // Wait for the end of the autonomous period
                            while (opModeIsActive()) {
                                telemetry.addData("Robot Heading", drive.getPoseEstimate().getHeading());
                                telemetry.update();
                            }

                            sleep(2000);

                             break;

                        }
                        else if (recognition.getLabel().equals("Smile")) {
                            // Smile has been detected, parking position 3

                            break;
                        }
                        else {

                            break;
                        }

                    }
                }
                telemetry.update();
                if(Math.round(timelapse.seconds())>10.00){
                    TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startingPose)
                            .strafeTo(new Vector2d(firstLowJunctionX,startingY))
                            .lineTo(new Vector2d(firstLowJunctionX, firstLowJunctionY))
                            .waitSeconds(.35)
                            .lineTo(new Vector2d(firstLowJunctionX, firstLowJunctionY - 10.00))
                            .strafeTo(new Vector2d(-64.00,firstLowJunctionY-7.00))
                            .splineToLinearHeading(new Pose2d(stackX + 5.00, stackY, stackHeading), Math.toRadians(80.00))
                            .lineTo(new Vector2d(stackX - 1.0, stackY))
                            .lineTo(new Vector2d(stackX, stackY))
                            .addTemporalMarker(1.0, () ->
                            {
                                moveSlider(lowJunctionPosition);
                            })
                            .addTemporalMarker(2.75, () ->
                            {
                                openGripper();
                            })
                            .addTemporalMarker(4.0, () ->
                            {
                                moveSlider(stackTopCone);
                            })


                            .build();

                    drive.followTrajectorySequence(seq1);
                    //Grab top cone from stack and place it in low junction.
                    Pose2d secondStartingPose = new Pose2d(stackX - 1.0, stackY, stackHeading);
                    TrajectorySequence seq2 = drive.trajectorySequenceBuilder(secondStartingPose)
                            .waitSeconds(0.15)
                            .lineTo(new Vector2d(stackX+2.0,stackY))
                            .lineTo(new Vector2d(-45.00,stackY))
                            //  .lineToLinearHeading(new Pose2d(-50.00, -20.00, Math.toRadians(270)))
                            .lineToLinearHeading(new Pose2d(-55.00, stackY, Math.toRadians(270)))
                            .lineTo(new Vector2d(firstLowJunctionXBack, firstLowJunctionYBack))
                            .addTemporalMarker(0.0, () ->
                            {
                                closeGripper();
                            })
                            .addTemporalMarker(0.15, () ->
                            {
                                moveSlider(lowJunctionPosition);
                            })
                            .addTemporalMarker(1.5, () ->
                            {
                                openGripper();
                            })
                            .build();
                    drive.followTrajectorySequence(seq2);

                    // Wait for the end of the autonomous period
                    while (opModeIsActive()) {
                        telemetry.addData("Nothing read... Failsafe Autonomous routine X Pose", drive.getPoseEstimate().getX());
                        telemetry.addData("Nothing read... Failsafe Autonomous routine Y Pose", drive.getPoseEstimate().getX());
                        telemetry.addData("Robot Heading", drive.getPoseEstimate().getHeading());
                        telemetry.update();
                    }
                }
            }
        }
        // Deactivate TFOD.
        tfod.deactivate();

        vuforiaPOWERPLAY.close();
        tfod.close();
    }

    /**
     * Display info (using telemetry) for a recognized object.
     */
    private void displayInfo(int i) {
        // Display label info.
        // Display the label and index number for the recognition.
        telemetry.addData("label " + i, recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        telemetry.addData("Left, Top " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        telemetry.addData("Right, Bottom " + i, Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
    }

    /**
     * Describe this function...
     */
    private void moveSlider(int sliderTargetPosition) {
        slider.setTargetPosition(sliderTargetPosition);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.75);
    }

    private void closeGripper(){
        gripper.setPosition(1.0);
    }
    private void openGripper(){
        gripper.setPosition(0.0);
    }


}