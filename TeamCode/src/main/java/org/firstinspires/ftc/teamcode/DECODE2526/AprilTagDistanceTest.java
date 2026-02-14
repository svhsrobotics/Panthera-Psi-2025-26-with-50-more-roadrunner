package org.firstinspires.ftc.teamcode.DECODE2526;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
public class AprilTagDistanceTest extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private YawPitchRollAngles cameraOrientation;
    private Position cameraPosition;
    private DcMotor right;
    private DcMotor left;
    private ArrayList detections;
    //where the robot is targeting
    //According to google, the red goal is at (-58.3727, 55.6425, 29.5).
    // the blue goal is at -58.3727, 55.6425, 29.5)
    // i don't know if this is right
    double targetingx = -58.3727;
    double targetingy = 55.6435;
    //ok

    @Override

    public void runOpMode() throws InterruptedException {
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        cameraPosition = new Position(DistanceUnit.INCH,
                0, 8, 0, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -90 + 19, 0, 0);
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


        waitForStart();

        while (opModeIsActive()) {
            right.setPower((gamepad1.right_stick_x + gamepad1.left_stick_y));
            left.setPower((gamepad1.right_stick_x - gamepad1.left_stick_y));

            telemetry.addData("hi",visionPortal.getCameraState());

            if (!aprilTag.getDetections().isEmpty()) {
                //run through all detections, making sure theyre not null
                for (int i = 0; i < detections.size(); i++) {
                    if (aprilTag.getDetections().get(i)!=null) {
                        double botx = aprilTag.getDetections().get(i).robotPose.getPosition().x;
                        double boty = aprilTag.getDetections().get(i).robotPose.getPosition().y;
                        telemetry.addData("botX", botx);
                        telemetry.addData("boty", boty);

                        //get distance from the target using the pythagorean theorum
                        double lengthx = targetingx - botx;
                        double lengthz = targetingy - boty;
                        double distancefromtarget = Math.hypot(lengthx, lengthz);
                        telemetry.addData("distance", distancefromtarget);
                    }
                }
            }

            telemetry.update();

        }
    }
}