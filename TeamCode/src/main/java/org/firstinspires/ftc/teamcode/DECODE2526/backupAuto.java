package org.firstinspires.ftc.teamcode.DECODE2526;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous
public class backupAuto extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        left.setPower(-.5);
        right.setPower(.5);
        sleep(1000);

    }
}
