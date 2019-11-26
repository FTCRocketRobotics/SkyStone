package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Autonomous
public class StartPos1R extends LinearOpMode {

    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;
    private DcMotor arm;

    @Override
    public void runOpMode(){

        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        arm=hardwareMap.get(DcMotor.class,"arm");

        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();


        while (opModeIsActive()){

            double x = 0;
            double y = 0;
            double turn = 0;
            double power= 0;
            boolean pullIn = false;
            boolean pullOut = false;

            ElapsedTime eTime = new ElapsedTime();

            //go straight ahead
            x = 0.0;
            y = 1.0;
            turn = 0;
            power = 0.8;
            pullIn = false;
            pullOut = false;

            Movement first = new Movement(x,y,turn,power,pullIn,pullOut);

            fL.setPower(first.lF_power);
            fR.setPower(first.rF_power);
            bL.setPower(first.lB_power);
            bR.setPower(first.rB_power);
            arm.setPower(first.arm_power);

            eTime.reset();
            while (eTime.time()< 1.5) {}


        }



    }


}
