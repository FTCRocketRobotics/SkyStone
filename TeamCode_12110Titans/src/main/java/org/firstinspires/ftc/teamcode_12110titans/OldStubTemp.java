package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OldStubTemp extends LinearOpMode {

    //We love Linnaea's OpModes
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

    // private DcMotor arm;

    private DcMotor elevator;
    private DcMotor spinny;
    private DcMotor inAndOut;

    //limit switch
    private DigitalChannel jackieChan;


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

    static final double MAX_EXTENSION_LIMIT_INANDOUT = 0.2 * COUNTS_PER_MOTOR_REV_ANDYMARK;
    static final double MIN_EXTENSION_LIMIT_INANDOUT =-5.0 * COUNTS_PER_MOTOR_REV_ANDYMARK;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){

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

        jackieChan = hardwareMap.get(DigitalChannel.class,"jackieChan");


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

        jackieChan.setMode(DigitalChannel.Mode.INPUT);

        //Wait for driver to press PLAY
        waitForStart();

        //run until the driver presses STOP
        while(opModeIsActive()){


        }


    }



}
