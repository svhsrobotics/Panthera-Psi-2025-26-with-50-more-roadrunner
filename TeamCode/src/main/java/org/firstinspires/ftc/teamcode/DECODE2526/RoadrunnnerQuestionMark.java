package org.firstinspires.ftc.teamcode.DECODE2526;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@Config
@Autonomous(name = "silly billy7", group = "autonomous")
public class RoadrunnnerQuestionMark extends LinearOpMode {


    public class Shooter {
        private DcMotor launch1;
        private DcMotor launch2;

        public Shooter(HardwareMap hardwareMap) {
            launch2 = hardwareMap.get(DcMotor.class, "launch2");
            launch1 = hardwareMap.get(DcMotor.class, "launch1");
            launch1.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public class Gate {
        private Servo gate1;
        private Servo gate2;

        public Gate(HardwareMap hardwareMap) {
            gate1 = hardwareMap.get(Servo.class, "gateServo");
            gate2 = hardwareMap.get(Servo.class, "gateServo2");
            gate2.setDirection(Servo.Direction.REVERSE);
        }
        public class OpenGate implements Action {
            //you can do studd here if you want
            int openPos = 1;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                gate1.setPosition(openPos);
                gate2.setPosition(openPos);
                return false; //true repeats the loop, false ends it.

            }
        }
        public Action openGate(){
            return new OpenGate();
        }
        public class CloseGate implements Action{
            int closePos = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket){
                gate1.setPosition(closePos);
                gate2.setPosition(closePos);
                return false;
            }
            public Action closeGate(){
                return new CloseGate();
            }
        }
    }
    //end of the gate stuff
double startX = 0;
    double startY = 0;
    double startAng = 0;
    @Override
    public void runOpMode(){
        Pose2d initPose = new Pose2d(startX, startY, startAng);
        TankDrive drive = new TankDrive(hardwareMap, initPose);

        Gate gate = new Gate(hardwareMap); //makes a new gate (lol)
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initPose)

                        .turn(Math.PI)
                                .lineToX(-50)
                                        .turn(Math.toRadians(90));
        waitForStart();
if(isStopRequested()) {return;}
        Actions.runBlocking(new SequentialAction(
                tab1.build(),
                gate.openGate()
        ));
    }



}
