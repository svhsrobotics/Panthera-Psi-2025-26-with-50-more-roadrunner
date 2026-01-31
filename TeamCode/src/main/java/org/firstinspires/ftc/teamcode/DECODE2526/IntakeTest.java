package org.firstinspires.ftc.teamcode.DECODE2526;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Debouncer;

@TeleOp
public class IntakeTest extends LinearOpMode {


    private DcMotor intake;
    private Debouncer debouncer = new Debouncer();
    private Debouncer downbouncer = new Debouncer();
    double servoPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
intake = hardwareMap.get(DcMotor.class, "intake");
intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
waitForStart();
while (opModeIsActive()){
    if(debouncer.update(gamepad1.dpad_up)){
        servoPos++;
        System.out.println(servoPos);
    } else if(downbouncer.update(gamepad1.dpad_down)){
        servoPos++;
        System.out.println(servoPos);
    }
intake.setTargetPosition((int)servoPos);
    intake.setPower(1);
    intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//~= 750 is a full rotation


}
    }
}
