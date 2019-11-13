package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

@TeleOp
public class OneDriverStrategyOne extends LinearOpMode {

    //We love Linnaea's OpModes
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

    @Override
    public void runOpMode() {
        telemetry.addData("Status","Initialized");
        telemetry.update();

        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");

        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);

        //Wait for driver to press PLAY
        waitForStart();

        //run until the driver presses STOP
        while(opModeIsActive()){

            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            double direction_travel;
            double direction_wheels;

            double lF_power;
            double rF_power;
            double lB_power;
            double rB_power;

            direction_travel=Math.atan2(y,x);
            direction_wheels=direction_travel-Math.PI/4;

            lF_power=Math.cos(direction_wheels);
            rF_power=Math.sin(direction_wheels);
            lB_power=Math.sin(direction_wheels);
            rB_power=Math.cos(direction_wheels);

            if((x==0 && y==0) || gamepad1==null ){
                fL.setPower(0);
                fR.setPower(0);
                bL.setPower(0);
                bR.setPower(0);
            }else{
                fL.setPower(lF_power);
                fR.setPower(rF_power);
                bL.setPower(lB_power);
                bR.setPower(rB_power);

            }

            telemetry.addData("fl",lF_power);
            telemetry.addData("fR",rF_power);
            telemetry.addData("bL",lB_power);
            telemetry.addData("bR",rB_power);

            

            telemetry.update();
        }




    }


}

