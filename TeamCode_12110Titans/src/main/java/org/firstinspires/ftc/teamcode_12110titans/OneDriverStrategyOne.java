package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@TeleOp
public class OneDriverStrategyOne extends LinearOpMode {

    //We love Linnaea's OpModes
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

   // private DcMotor arm;

    private DcMotor elevator;
    private DcMotor spinny;
    private DcMotor inAndOut;


    static final double COUNTS_PER_MOTOR_REV_GOBUILDA = 383.06;    // // eg: GoBuilda Motor Encoder
    static final double COUNTS_PER_MOTOR_REV_ANDYMARK = 1120.00;    // // eg: AndyMark neverest 40
    /*static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    //static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_ANDYMARK * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = .5;
    static final double TURN_SPEED = .5;*/

    static final double MAX_EXTENSION_LIMIT_ELEVATOR = 1.0 * COUNTS_PER_MOTOR_REV_GOBUILDA;
    static final double MIN_EXTENSION_LIMIT_ELEVATOR =-1.0 * COUNTS_PER_MOTOR_REV_GOBUILDA;

    static final double MAX_EXTENSION_LIMIT_INANDOUT = 1.0 * COUNTS_PER_MOTOR_REV_ANDYMARK;
    static final double MIN_EXTENSION_LIMIT_INANDOUT =-1.0 * COUNTS_PER_MOTOR_REV_ANDYMARK;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status","Initialized");
        telemetry.update();

        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        //arm = hardwareMap.get(DcMotor.class, "arm");

        elevator  = hardwareMap.get(DcMotor.class,"elevator");
        spinny = hardwareMap.get(DcMotor.class,"spinny");
        inAndOut = hardwareMap.get(DcMotor.class,"inAndOut");


        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);

        spinny.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        inAndOut.setDirection(DcMotor.Direction.FORWARD);

        inAndOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        inAndOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Wait for driver to press PLAY
        waitForStart();

        //run until the driver presses STOP
        while(opModeIsActive()){

            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn=gamepad1.right_stick_x;
            //boolean pullIn=gamepad1.dpad_down;
           // boolean pullOut=gamepad1.dpad_up;

            boolean elevatorUp =gamepad1.dpad_up;
            boolean elevatorDown = gamepad1.dpad_down;
            boolean in =gamepad1.dpad_left;
            boolean out =gamepad1.dpad_right;
            boolean pullin =gamepad1.y;
            boolean pushOut =gamepad1.a;

            double direction_travel;
            double direction_wheels;

            double lF_power;
            double rF_power;
            double lB_power;
            double rB_power;

            double elevator_power;
            double inAndOut_power;
            double spinny_power;

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

            if((x==0 && y==0) || gamepad1==null ){
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

            if(in && inAndOut.getCurrentPosition() < MAX_EXTENSION_LIMIT_INANDOUT && inAndOut.getCurrentPosition() > MIN_EXTENSION_LIMIT_INANDOUT){
                inAndOut_power=0.5;
            }else if (out && inAndOut.getCurrentPosition() < MAX_EXTENSION_LIMIT_INANDOUT && inAndOut.getCurrentPosition() > MIN_EXTENSION_LIMIT_INANDOUT){
                inAndOut_power=-0.5;
            }else{
                inAndOut_power=0.0;
            }
            inAndOut.setPower(inAndOut_power);


            if(elevatorUp && elevator.getCurrentPosition() < MAX_EXTENSION_LIMIT_ELEVATOR && elevator.getCurrentPosition() > MIN_EXTENSION_LIMIT_ELEVATOR ){
                elevator_power=0.5;
            }else if (elevatorDown && elevator.getCurrentPosition() < MAX_EXTENSION_LIMIT_ELEVATOR && elevator.getCurrentPosition() > MIN_EXTENSION_LIMIT_ELEVATOR ){
                elevator_power=-0.5;
            }else{
                elevator_power=0.0;
            }
            elevator.setPower(elevator_power);

            if(pullin){
                spinny_power=0.5;
            }else if (pushOut){
                spinny_power=-0.5;
            }else{
                spinny_power=0.0;
            }
            spinny.setPower(spinny_power);



            telemetry.addData("fl",lF_power);
            telemetry.addData("fR",rF_power);
            telemetry.addData("bL",lB_power);
            telemetry.addData("bR",rB_power);
            //telemetry.addData("arm",arm_power);
            

            telemetry.update();
        }




    }


}

/** © All Rights Reserved */