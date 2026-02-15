package org.firstinspires.ftc.teamcode.DECODE2526;

import androidx.core.widget.TextViewCompat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class autoNOSCHEDULERS extends LinearOpMode {
private DcMotor left;
private DcMotor right;
private DcMotor launch;
private DcMotor launch2;
private Servo gateServo;
private Servo gateServo2;
    private DcMotor intake;
    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        launch2 = hardwareMap.get(DcMotor.class, "launch2");
        launch2.setDirection(DcMotorSimple.Direction.REVERSE);
        launch = hardwareMap.get(DcMotor.class, "launch1");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo2 = hardwareMap.get(Servo.class, "gateServo2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
right.setPower(0.998);
left.setPower(1);
wait(100);
System.out.println("1");
right.setPower(0);
left.setPower(0);
launch.setPower(1);
launch2.setPower(1);
wait(6000);
        System.out.println("2");
intake.setPower(0.2);
wait(300);
        System.out.println("3");
gateServo.setPosition(1);
gateServo2.setPosition(1);
wait(1000);
        System.out.println("4");
gateServo.setPosition(0.09);
gateServo2.setPosition(0.09);
launch.setPower(0);
launch2.setPower(0);
    }
}
