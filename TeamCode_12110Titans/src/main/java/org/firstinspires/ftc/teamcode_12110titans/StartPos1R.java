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

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: GoBuilda Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ElapsedTime     runtime = new ElapsedTime();

    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;
    //private DcMotor arm;

    @Override
    public void runOpMode(){

        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        //arm=hardwareMap.get(DcMotor.class,"arm");

        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
       //arm.setDirection(DcMotor.Direction.FORWARD);

        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);

       /* waitForStart();


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
            turn = 0.0;
            power = 0.8;
            pullIn = false;
            pullOut = false;

            Movement first = new Movement(x,y,turn,power,pullIn,pullOut);

            fL.setPower(first.lF_power);
            fR.setPower(first.rF_power);
            bL.setPower(first.lB_power);
            bR.setPower(first.rB_power);
            //arm.setPower(first.arm_power);

            eTime.reset();
            while (eTime.time()< 1.5) {}

            //go right
            x = 1.0;
            y = 0.0;
            turn = 0.0;
            power = 0.8;
            pullIn = false;
            pullOut = false;

            Movement second = new Movement(x,y,turn,power,pullIn,pullOut);

            fL.setPower(second.lF_power);
            fR.setPower(second.rF_power);
            bL.setPower(second.lB_power);
            bR.setPower(second.rB_power);
            //arm.setPower(second.arm_power);

            eTime.reset();
            while (eTime.time()< 1.5) {}

            //go backwards
            x = 0.0;
            y = -1.0;
            turn = 0.0;
            power = 0.8;
            pullIn = false;
            pullOut = false;

            Movement third = new Movement(x,y,turn,power,pullIn,pullOut);

            fL.setPower(third.lF_power);
            fR.setPower(third.rF_power);
            bL.setPower(third.lB_power);
            bR.setPower(third.rB_power);
            //arm.setPower(third.arm_power);

            eTime.reset();
            while (eTime.time()< 1.5) {}

            //Im going leeaft
            x = -1.0;
            y = 0.0;
            turn = 0.0;
            power = 0.8;
            pullIn = false;
            pullOut = false;

            Movement fourth = new Movement(x,y,turn,power,pullIn,pullOut);

            fL.setPower(fourth.lF_power);
            fR.setPower(fourth.rF_power);
            bL.setPower(fourth.lB_power);
            bR.setPower(fourth.rB_power);
            //arm.setPower(fourth.arm_power);

            eTime.reset();
            while (eTime.time()< 1.5) {}

            fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        }*/



    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = fL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = fR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            fL.setTargetPosition(newLeftTarget);
            fR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fL.setPower(Math.abs(speed));
            fR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fL.isBusy() && fR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        fL.getCurrentPosition(),
                        fR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            fL.setPower(0);
            fR.setPower(0);

            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
