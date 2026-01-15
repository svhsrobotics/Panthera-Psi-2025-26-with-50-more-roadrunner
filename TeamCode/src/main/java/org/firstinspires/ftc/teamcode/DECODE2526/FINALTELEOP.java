package org.firstinspires.ftc.teamcode.DECODE2526;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.Toggle;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

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
    private Toggle toggle = new Toggle();

    ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();

private Debouncer shootDebouncer = new Debouncer();
    private boolean debounce;
    private boolean isthethingthething;
    double totalCurrent = 0;
    int denominator = 0;
    double averageCurrent = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        launch2=hardwareMap.get(DcMotor.class, "launch2");
        launch=hardwareMap.get(DcMotor.class, "launch1");
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo2 = hardwareMap.get(Servo.class, "gateServo2");
        voltSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        frontLights = hardwareMap.get(RevBlinkinLedDriver.class, "frontLights");
        rearLights = hardwareMap.get(RevBlinkinLedDriver.class, "rearLights");
AtomicBoolean shooting = new AtomicBoolean(false);
        gateServo2.setDirection(Servo.Direction.REVERSE);
        debounce=true;
        isthethingthething=false;
        Debouncer debouncingOnDeesNuts = new Debouncer();
        Debouncer debouncer2 = new Debouncer();
        double gatePos = 0;
        double launchpower = 0;
        System.out.println("set gatePos to 0");
        double servoshootpos = 0.25;

        while(opModeInInit()){
            launchpower=0.9;
            frontLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            rearLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
        waitForStart();
        while (opModeIsActive()) {

            System.out.println("gatepos: " + gatePos);
            System.out.println("servo 1 pos: " + gateServo.getPosition());
            System.out.println("servo 2 pos: " + gateServo2.getPosition());
            System.out.println("kickpos: " + servoshootpos);


            right.setPower((gamepad1.right_stick_x + gamepad1.left_stick_y));
            left.setPower((gamepad1.right_stick_x - gamepad1.left_stick_y));
            //0.25 is open 0.02 is close
            if (gamepad1.dpad_up) { //opens da gate
                gateServo.setPosition(servoshootpos);
                gateServo2.setPosition(servoshootpos);
            } else if (gamepad1.dpad_down) { //close
                gateServo.setPosition(0.015);
                gateServo2.setPosition(0.015);
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
            telemetry.addData("average milliamp",  averageCurrent);
            telemetry.addData("shootservopos",  servoshootpos);
            telemetry.update();

            if(gamepad1.b){
                intake.setPower(1);
                gateServo.setPosition(.015);
                gateServo2.setPosition(.015);
                launch.setPower(0);
                launch2.setPower(0);
            }

            if(gamepad1.right_bumper){
                intake.setPower(0);


            }


            if(shootDebouncer.update(gamepad1.a) && !shooting.get()){

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
                    gateServo.setPosition(0.25);
                    gateServo2.setPosition(0.25);
                }, 5, TimeUnit.SECONDS);

                scheduler.schedule(() ->{
                    gateServo.setPosition(0.015);
                    gateServo2.setPosition(0.015);
                    intake.setPower(0);
                    launch.setPower(0);
                    launch2.setPower(0);
                    shooting.set(false);
                }, 6, TimeUnit.SECONDS);



            }

            if(gamepad2.y){
                intake.setPower(1);
            }

            if(gamepad2.x){
                intake.setPower(0);
            }
            if(gamepad2.a){
                intake.setPower(-1);
            }

            if(gamepad2.dpad_left){
                servoshootpos = servoshootpos+1;
            }
            if(gamepad2.dpad_right){
                servoshootpos = servoshootpos-1;
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
        }
        scheduler.shutdownNow();
    }

}

