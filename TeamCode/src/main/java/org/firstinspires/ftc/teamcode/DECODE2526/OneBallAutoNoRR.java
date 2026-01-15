package org.firstinspires.ftc.teamcode.DECODE2526;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous

public class OneBallAutoNoRR extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private YawPitchRollAngles cameraOrientation;
    private Position cameraPosition;

    private double cameraX = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        double driveToTicks = 0;
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "left");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "right");
        DcMotor shoot = hardwareMap.get(DcMotor.class, "launch1");
        DcMotor  shoot2 = hardwareMap.get(DcMotor.class, "launch2");
        DcMotor intake = hardwareMap.get(DcMotorEx.class, "intake");
       Servo  gateServo = hardwareMap.get(Servo.class, "gateServo");
       Servo gateServo2 = hardwareMap.get(Servo.class, "gateServo2");
        shoot2.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        cameraPosition = new Position(DistanceUnit.INCH,
                0, 8, 0, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -90 + 19, 0, 0);

        waitForStart();

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();


        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        // end method initAprilTag()


            while(left.getCurrentPosition() < 3000 ) {
                telemetry.addData("leftpos", left.getCurrentPosition());
                telemetry.addData("rightpos", right.getCurrentPosition());
                telemetry.update();
                left.setPower(-1);
                right.setPower(-1);
                left.setTargetPosition(3000);
                right.setTargetPosition(3000);
                left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("target, ", right.getTargetPosition());
            }
        left.setPower(0);
        right.setPower(0);
        sleep(500);
        System.out.println("reseting encoders");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        System.out.println("encoders positions are: " + left.getCurrentPosition() + " and " + right.getCurrentPosition());
            while(left.getCurrentPosition() < 300 ){

                left.setPower(1);
                right.setPower(-1);
                left.setTargetPosition(300);
                right.setTargetPosition(-300);
                left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

          /*  while(true == true){
                left.setPower(0);
                right.setPower(0);
                telemetry.addData("leftpos", left.getCurrentPosition());
                telemetry.addData("rightpos", right.getCurrentPosition());
                telemetry.update();
            }*/



        while ((cameraX < 250 || cameraX > 350) && aprilTag.getDetections() != null && gamepad1.dpad_down && !aprilTag.getDetections().isEmpty()) {
            cameraX = aprilTag.getDetections().get(0).center.x;
            if (cameraX > 275) {
               left.setPower(.2);
            }
            if (cameraX < 325) {
               right.setPower(.2);
            }


        }
        if (aprilTag.getDetections() != null && !aprilTag.getDetections().isEmpty()) {
            ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("CENTER", detections.get(0).center.x);
            telemetry.addData("BEARING", detections.get(0).ftcPose.bearing);

        } else {
            telemetry.addData("CENTER", "NULL");
            telemetry.addData("BEARING", "NULL");
        }
        telemetry.update();
        shoot.setPower(1);
        shoot2.setPower(1);
        sleep(5000);
        intake.setPower(.2);

       gateServo.setPosition(1);
       gateServo2.setPosition(-1);
       sleep(5000);


    }


           // telemetry.addData("end",0);
            //telemetry.update();









    }

