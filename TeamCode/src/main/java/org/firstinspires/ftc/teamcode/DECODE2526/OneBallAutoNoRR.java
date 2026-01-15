package org.firstinspires.ftc.teamcode.DECODE2526;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class OneBallAutoNoRR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double driveToTicks = 0;
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "left");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();


            while(left.getCurrentPosition() < 10000 && right.getCurrentPosition() < 1000) {
                telemetry.addData("leftpos", left.getCurrentPosition());
                telemetry.addData("rightpos", right.getCurrentPosition());
                telemetry.update();
                left.setPower(-1);
                right.setPower(-1);
                left.setTargetPosition(10000);
                right.setTargetPosition(10000);
                left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("target, ", right.getTargetPosition());
            }

            while(left.getCurrentPosition() < 1000 && right.getCurrentPosition() > -1000){
                left.setTargetPosition(100);
                right.setTargetPosition(-100);
            }
            telemetry.addData("end",0);
            telemetry.update();









    }
}
