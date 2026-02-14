package org.firstinspires.ftc.teamcode.DECODE2526;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous

public class OneBallAutoNoRRBlue extends LinearOpMode {

    private final Toggle toggle = new Toggle();
    private final Debouncer shootDebouncer = new Debouncer();
    private final Debouncer reallyCoolDebouncer = new Debouncer();
    private final double cameraX = 0;
    ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    double totalCurrent = 0;
    double closePos = 0.09;
    int denominator = 0;
    double averageCurrent = 0;
    private DcMotor right;
    private DcMotor left;
    private DcMotor launch2;
    private DcMotor launch;
    private DcMotorEx intake;
    private Servo gateServo;
    private Servo gateServo2;
    private VoltageSensor voltSensor;
    private RevBlinkinLedDriver frontLights;
    private RevBlinkinLedDriver rearLights;
    private Servo siloServo;
    private boolean debounce;
    private boolean isthethingthething;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private YawPitchRollAngles cameraOrientation;
    private Position cameraPosition;

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
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        AtomicBoolean siloshootig = new AtomicBoolean(false);
        AtomicInteger silopos = new AtomicInteger(1000);
right.setDirection(DcMotorSimple.Direction.REVERSE);





       waitForStart();

           left.setPower(0.985);
           right.setPower(1);
           sleep(600);
           left.setPower(0);
           right.setPower(0);

            gateServo.setPosition(closePos);
            gateServo2.setPosition(closePos);
            shooting.set(true);
        while ( shooting.get()) { //shoot ball that is currently primed

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
scheduler.shutdownNow();

    }


}
