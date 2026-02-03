package org.firstinspires.ftc.teamcode.DECODE2526;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.Toggle;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@TeleOp
public class FINALTELEOP extends LinearOpMode{
    private DcMotor right;
    private DcMotor left;
    private DcMotor launch2;
    private DcMotor launch;
    private DcMotorEx intake;
    private Servo gateServo;
    private Servo gateServo2;
    private VoltageSensor  voltSensor;
    private RevBlinkinLedDriver frontLights;
    private RevBlinkinLedDriver rearLights;
    private Servo siloServo;
    private Toggle toggle = new Toggle();

    ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();

private Debouncer shootDebouncer = new Debouncer();
private Debouncer reallyCoolDebouncer = new Debouncer();
    private boolean debounce;
    private boolean isthethingthething;
    double totalCurrent = 0;
    double closePos = 0.09;
    int denominator = 0;
    double averageCurrent = 0;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private YawPitchRollAngles cameraOrientation;
    private Position cameraPosition;
private double cameraX = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        launch2 = hardwareMap.get(DcMotor.class, "launch2");
        launch = hardwareMap.get(DcMotor.class, "launch1");
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo2 = hardwareMap.get(Servo.class, "gateServo2");
        voltSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        frontLights = hardwareMap.get(RevBlinkinLedDriver.class, "frontLights");
        rearLights = hardwareMap.get(RevBlinkinLedDriver.class, "rearLights");
        siloServo = hardwareMap.get(Servo.class, "siloServo");
        AtomicBoolean shooting = new AtomicBoolean(false);
        gateServo2.setDirection(Servo.Direction.REVERSE);
        debounce = true;
        isthethingthething = false;
        Debouncer debouncingOnDeesNuts = new Debouncer();
        Debouncer debouncer2 = new Debouncer();
        double gatePos = 0;
        double launchpower = 0;
        //System.out.println("set gatePos to 0");
        double servoshootpos = 0.5;

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

        while (opModeInInit()) {
            launchpower = 0.9;
            frontLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            rearLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

        waitForStart();


intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
AtomicBoolean siloshootig = new AtomicBoolean(false);

AtomicInteger silopos = new AtomicInteger(1000);

        while (opModeIsActive()) {

            telemetry.addData("intakePos", intake.getCurrentPosition());



           if(intake.getCurrentPosition() >=750) {
               intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           }

           System.out.println(intake.getCurrentPosition() + " wheel pos");
           System.out.println(intake.getCurrentPosition()/120 + " pos/120");

                if (gamepad1.a && (intake.getCurrentPosition() / 120 >= 4 && intake.getCurrentPosition()/120 <= 7) && !siloshootig.get()) {
                    intake.setPower(0);
                    sleep(1000);
                    siloshootig.set(true);
                    gateServo.setPosition(closePos);
                    gateServo2.setPosition(closePos);

                    scheduler.schedule(()->{
                        siloServo.setPosition(0);
                        intake.setPower(0);
                    }, 100, TimeUnit.MILLISECONDS);

                    double finalServoshootpos = servoshootpos;
                    scheduler.schedule(() -> {
                        gateServo.setPosition(finalServoshootpos);
                        gateServo2.setPosition(finalServoshootpos);

                        siloServo.setPosition(0.4);
                        siloshootig.set(false);
                    }, 3000 , TimeUnit.MILLISECONDS);
                }





            if (intake.getPower() != 0) {
                rearLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else {
                rearLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            if (launchpower != 0) {
                frontLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else {
                frontLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

           /* System.out.println("gatepos: " + gatePos);
            System.out.println("servo 1 pos: " + gateServo.getPosition());
            System.out.println("servo 2 pos: " + gateServo2.getPosition());
            System.out.println("kickpos: " + servoshootpos);*/


            right.setPower((gamepad1.right_stick_x + gamepad1.left_stick_y));
            left.setPower((gamepad1.right_stick_x - gamepad1.left_stick_y));
            //0.25 is open 0.02 is close
            if (gamepad1.dpad_up) { //opens da gate
                gateServo.setPosition(servoshootpos);
                gateServo2.setPosition(servoshootpos);
            } else if (gamepad1.dpad_down) { //close
                gateServo.setPosition(closePos);
                gateServo2.setPosition(closePos);
            }

            if (gamepad1.dpad_right) { //dpad manually spins up or down flywheels
                launch.setPower(launchpower);
                launch2.setPower(launchpower);


            } else if (gamepad1.dpad_left) {
                launch.setPower(0);
                launch2.setPower(0);

            }

            if (debouncingOnDeesNuts.update(gamepad2.dpad_up)) {
                launchpower += .01;
            } else if (debouncer2.update(gamepad2.dpad_down)) {
                launchpower -= .01;
            }

            telemetry.addData("shoot power", launchpower);
            telemetry.addData("servo1Pos: ", gateServo.getPosition());
            telemetry.addData("servo2Pos", gateServo2.getPosition());
            telemetry.addData("average milliamp", averageCurrent);
            telemetry.addData("shootservopos", servoshootpos);
            telemetry.update();

            if (gamepad1.b) {
                intake.setPower(1);
                gateServo.setPosition(closePos);
                gateServo2.setPosition(closePos);
               // launch.setPower(0);
                //launch2.setPower(0);
            }

            if (gamepad1.right_bumper) {
                intake.setPower(0);
            }


            if (shootDebouncer.update(gamepad2.a) && !shooting.get()) { //shoot ball that is currently primed

                //shoot
                /*
                spin up wheel
                open gate
                wait a little
                set power 0
                 */
                shooting.set(true);
                intake.setPower(0.2);
                launch.setPower(1);
                launch.setPower(1);
                //wait 1 sec then open the gate
                scheduler.schedule(() -> {
                    gateServo.setPosition(1);
                    gateServo2.setPosition(1);
                }, 4, TimeUnit.SECONDS);

                scheduler.schedule(() -> {
                    gateServo.setPosition(closePos);
                    gateServo2.setPosition(closePos);
                   // intake.setPower(0);
                   // launch.setPower(0);
                    //launch2.setPower(0);
                    shooting.set(false);
        }, 5, TimeUnit.SECONDS);


            }

            if (gamepad2.y) {
                launch.setPower(0);
                launch2.setPower(0);
            }

            if (gamepad2.x) {
                scheduler.shutdownNow();
            }



            if (gamepad2.dpad_left) {
                servoshootpos = servoshootpos + 1;
            }
            if (gamepad2.dpad_right) {
                servoshootpos = servoshootpos - 1;
            }


            // if ((intake.getCurrent(CurrentUnit.MILLIAMPS) > averageCurrent * 3) ){
            //    frontLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            //    rearLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            // } else if(intake.getCurrent(CurrentUnit.MILLIAMPS) > 5) {
            //    totalCurrent += intake.getCurrent(CurrentUnit.MILLIAMPS);
            //    denominator += 1;
            //  averageCurrent = totalCurrent/denominator;
            //}

            // if (voltSensor.getVoltage() < 11.5) {
            //    telemetry.addLine("YOUR VOLTAGE IS LOW");
            //   telemetry.update();
            // }

           /* if (gamepad1.x) {
                while (((cameraX < 250 || cameraX > 350) && aprilTag.getDetections() != null && gamepad1.dpad_down && !aprilTag.getDetections().isEmpty()) && opModeIsActive()) {
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
            }*/
            if(gamepad2.dpad_left){ //estop
                intake.setPower(0);
                scheduler.shutdownNow();
                launchpower = 0;
                left.setPower(0);
                right.setPower(0);
            }


        }
        scheduler.shutdownNow();
    }

}



/*TODO: Gamepad 1 (Primary Driver)


    Intake Start (Cross/A Button): Logic Swap. * Code Reality: gamepad1.a triggers the shootDebouncer sequence (spinning flywheels and opening gates). It does not start a standard intake loop.

    Shoot in Motif Order (Circle/B Button): Logic Swap.

        Code Reality: gamepad1.b actually sets intake.setPower(1), effectively acting as the "Intake Start" button, while also closing the gates.

    Stop Intake (Triangle/Y Button): Missing. * Code Reality: The code does not reference gamepad1.y at all. Intake stop is currently handled by gamepad1.right_bumper.

    Drive (Sticks): Partially Commented Out.

        Note: The lines assigning right.setPower and left.setPower to the joysticks are currently commented out (Lines 169–170). The robot will not move in its current state.

Gamepad 2 (Secondary Operator)

    Load 1 Ball from Silo (Circle/B Button): Missing. * Code Reality: The code does not reference gamepad2.b.

    Intake Spin In (Triangle/Y Button): Functional, but lacks a "Stop" on the same controller besides the E-Stop.

    Intake Spin Out (Cross/A Button): Logic Swap. * Code Reality: gamepad2.a is mapped to intake.setPower(-1), which matches "Spin Out," but the diagram labels the Cross button as "Shoot ball thats primed."

    Shoot ball thats primed (Cross/A Button): Missing.*/

