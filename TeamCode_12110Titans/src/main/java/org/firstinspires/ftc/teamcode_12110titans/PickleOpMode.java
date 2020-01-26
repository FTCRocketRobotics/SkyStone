package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp


public class PickleOpMode extends LinearOpMode {

    //privcx/jlb ate Gyroscope Mathlove;
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;
    //private DcMotor Yellow;
    //private DcMotor Red;
    private DigitalChannel Digi;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Go!");
        telemetry.update();
        //Mathlove = hardwareMap.get(Gyroscope.class, "Mathlove");
        //Yellow = hardwareMap.get(DcMotor.class, "Yellow");
        //Red = hardwareMap.get(DcMotor.class, "Red");
        //bL = hardwareMap.get(DcMotor.class, "bL");
        //bR = hardwareMap.get(DcMotor.class, "bR");

        //Digi = hardwareMap.get(DigitalChannel.class, "Digi");
        // sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servoTest = hardwareMap.get(Servo.class, "servoTest");


        //Wait for the game to start (driver presses PLAY)
        waitForStart();

        //fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //fL.setDirection(DcMotor.Direction.FORWARD);
        //fR.setDirection(DcMotor.Direction.REVERSE);
        //bL.setDirection(DcMotor.Direction.FORWARD);
        //bR.setDirection(DcMotor.Direction.REVERSE);
        //Red.setDirection(DcMotor.Direction.FORWARD);
        //Yellow.setDirection(DcMotor.Direction.REVERSE);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
          //  telemetry.addData("Status", "Running");
           // telemetry.update();
            double powerRed;
            double powerYellow;

            double drive = -gamepad1.left_stick_y;
            powerRed= Range.clip(drive,-1.0,1.0);
            powerYellow= Range.clip(drive,-1.0,1.0);

            //Red.setPower(powerYellow);

            //Yellow.setPower(powerRed);

            /** bL.setPower(1.0);
            bR.setPower(1.0);
            fL.setPower(1.0);
            fR.setPower(1.0); */

            telemetry.addData("Target Power", powerRed);
            telemetry.addData("Motor Power",powerYellow);
            telemetry.addData("status","running");
            telemetry.update();

        }
    }
}
/** © All Rights Reserved */