package org.firstinspires.ftc.teamcode.DECODE2526;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class OneBallAutoNoRR2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double driveToTicks = 0;
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "left");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("pos", left.getCurrentPosition());
            telemetry.addData("rightpos", right.getCurrentPosition());
            telemetry.update();
            left.setPower(1);
            right.setPower(1);
            sleep(100);
            left.setPower(0);
            right.setPower(0);





        }




    }
}
