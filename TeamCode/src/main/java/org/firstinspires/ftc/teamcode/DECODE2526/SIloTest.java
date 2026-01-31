package org.firstinspires.ftc.teamcode.DECODE2526;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Debouncer;

@TeleOp
public class SIloTest extends LinearOpMode {


    private Servo siloServo;
    private Debouncer debouncer = new Debouncer();
    private Debouncer downbouncer = new Debouncer();
    double servoPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
siloServo = hardwareMap.get(Servo.class, "siloServo");
waitForStart();
while (opModeIsActive()){
    if(debouncer.update(gamepad1.dpad_up)){
        servoPos = servoPos + 0.01;
        System.out.println(servoPos);
    } else if(downbouncer.update(gamepad1.dpad_down)){
        servoPos = servoPos - 0.01;
        System.out.println(servoPos);
    }

    siloServo.setPosition(servoPos);
    //.34 for mocing
    //0.5 for closed
}
    }
}
