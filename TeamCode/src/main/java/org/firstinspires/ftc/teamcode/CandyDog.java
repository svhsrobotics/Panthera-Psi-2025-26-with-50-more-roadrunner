package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
public class CandyDog extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        Toggle toggle = new Toggle();
        boolean isSlow = false;

        //DcMotor shootyMotor = hardwareMap.get(DcMotor.class, "shootyMotor);
        double rightXInput = 0;
        double leftInput = 0;
        //the amount the inputs gets divided by when in low sens mode
        double scale =  3.0;
        waitForStart();
        while(opModeIsActive()){
            rightXInput = gamepad1.right_stick_x; //TODO: change these if the robot drives backwards (or suck it up liam)
            leftInput = gamepad1.left_stick_y;
            telemetry.addLine("Controls: use sticks to move, and A to enable low sensitivity mode \n (maps the inputs using cube root curve)");
            telemetry.addData("low sens mode on?", isSlow);
            if(toggle.update(gamepad1.a)){
                rightXInput = Math.pow(rightXInput, 1/scale);
                leftInput = Math.pow(rightXInput, 1/scale);
                isSlow = true;
            } else{
                isSlow = false;
            }

            frontLeft.setPower((rightXInput+ leftInput)); //the logic from the psi decode tank drive
            backLeft.setPower((rightXInput + leftInput)); //but doubled bc there are 4 motors
            frontRight.setPower((rightXInput - leftInput));
            backRight.setPower((rightXInput - leftInput));
            telemetry.update();


        }
    }
}
