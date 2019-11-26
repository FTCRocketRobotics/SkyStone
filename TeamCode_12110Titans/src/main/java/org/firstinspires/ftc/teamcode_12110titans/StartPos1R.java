package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Autonomous
public class StartPos1R extends LinearOpMode {

    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

    @Override
    public void runOpMode(){

        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");

        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()){

            move(0.0,1.0,0.0, fL, fR, bL,fR);
        }



    }

    public static void move(double x, double y, double turn, DcMotor lF, DcMotor rF, DcMotor lB, DcMotor rB){


        double direction_travel;
        double direction_wheels;

        double lF_power;
        double rF_power;
        double lB_power;
        double rB_power;

        direction_travel=Math.atan2(y,x);
        direction_wheels=direction_travel-Math.PI/4;

        lF_power=Math.cos(direction_wheels)-turn;
        rF_power=Math.sin(direction_wheels)+turn;
        lB_power=Math.sin(direction_wheels)-turn;
        rB_power=Math.cos(direction_wheels)+turn;

        //A problem is that when you add power for direction and power for rotating, you can get power >1
        //So we had to adjust proportions
        if(Math.abs(lF_power)>1.0) {
            lF_power = lF_power / Math.abs(lF_power);
            rF_power = rF_power / Math.abs(lF_power);
            lB_power = lB_power / Math.abs(lF_power);
            rB_power = rB_power / Math.abs(lF_power);
        } else if (Math.abs(rF_power)>1.0) {
            lF_power = lF_power / Math.abs(rF_power);
            rF_power = rF_power / Math.abs(rF_power);
            lB_power = lB_power / Math.abs(rF_power);
            rB_power = rB_power / Math.abs(rF_power);
        } else if (Math.abs(lB_power)>1.0) {
            lF_power = lF_power / Math.abs(lB_power);
            rF_power = rF_power / Math.abs(lB_power);
            lB_power = lB_power / Math.abs(lB_power);
            rB_power = rB_power / Math.abs(lB_power);
        } else if (Math.abs(rB_power)>1.0) {
            lF_power = lF_power / Math.abs(rB_power);
            rF_power = rF_power / Math.abs(rB_power);
            lB_power = lB_power / Math.abs(rB_power);
            rB_power = rB_power / Math.abs(rB_power);
        } else {
            // do nothing
        }

        //Sometimes when your going straight (some exceptions0 the power to all the wheels is less then 1.

        //Also appreciate my wonderful code i made :)

        if(Math.abs(lF_power) < 1.0 && Math.abs(rF_power) < 1.0 && Math.abs(lB_power) < 1.0 && Math.abs(rB_power) < 1.0) {

            List<Double> powers = new ArrayList<Double>();

            powers.add(Math.abs(lF_power));
            powers.add(Math.abs(rF_power));
            powers.add(Math.abs(lB_power));
            powers.add(Math.abs(rB_power));

            double largest = Collections.max(powers);

            lF_power = lF_power / largest;
            rF_power = rF_power / largest;
            lB_power = lB_power / largest;
            rB_power = rB_power / largest;

        }

        if((x==0 && y==0)){
            fL.setPower(-turn);
            fR.setPower(turn);
            bL.setPower(-turn);
            bR.setPower(turn);
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
