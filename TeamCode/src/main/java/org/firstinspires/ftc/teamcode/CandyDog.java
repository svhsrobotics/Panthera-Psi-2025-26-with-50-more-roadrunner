package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class CandyDog extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        //DcMotor shootyMotor = hardwareMap.get(DcMotor.class, "shootyMotor);

        waitForStart();
        while(opModeIsActive()){
            frontLeft.setPower((gamepad1.right_stick_x + gamepad1.left_stick_y));
            backLeft.setPower((gamepad1.right_stick_x + gamepad1.left_stick_y));
            frontRight.setPower((gamepad1.right_stick_x - gamepad1.left_stick_y));
            backRight.setPower((gamepad1.right_stick_x - gamepad1.left_stick_y));


        }
    }
}
