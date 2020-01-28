package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BluePlatformWall extends LinearOpMode {

    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

    private DcMotor elevator;
    private DcMotor spinny;
    private DcMotor inAndOut;

    //limit switch
    private DigitalChannel jackieChan;

    static final double COUNTS_PER_MOTOR_REV_GOBUILDA = 383.06;    // // eg: GoBuilda Motor Encoder
    static final double COUNTS_PER_MOTOR_REV_ANDYMARK = 1120.00;    // // eg: AndyMark neverest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_ANDYMARK * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_GOBUILDA = (COUNTS_PER_MOTOR_REV_GOBUILDA * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //changed turn nd power 0.5 to 1.0
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 1.0;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");

        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);

        elevator  = hardwareMap.get(DcMotor.class,"elevator");
        spinny = hardwareMap.get(DcMotor.class,"spinny");
        inAndOut = hardwareMap.get(DcMotor.class,"inAndOut");

        jackieChan = hardwareMap.get(DigitalChannel.class,"jackieChan");

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinny.setDirection(DcMotor.Direction.FORWARD);
        elevator.setDirection(DcMotor.Direction.FORWARD);
        inAndOut.setDirection(DcMotor.Direction.FORWARD);

        inAndOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        inAndOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        jackieChan.setMode(DigitalChannel.Mode.INPUT);


        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                fL.getCurrentPosition(),
                fR.getCurrentPosition(),
                bL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();

        waitForStart();

        int distance = 0;

        //while (!isStopRequested()) {


        //changed power 0.8 to 1.0
        double x = -1.0;
        double y = 0.0;
        double turn = 0.0;
        double power = 1.0;
        boolean pullIn = false;
        boolean pullOut = false;

       /* Movement first = new Movement(x,y,turn,power,pullIn,pullOut);

        runtime.reset();
        while (runtime.seconds() < 0.2) {
            fL.setPower(first.lF_power*0.3);
            fR.setPower(first.rF_power*0.3);
            bL.setPower(first.lB_power*0.3);
            bR.setPower(first.rB_power*0.3);
        }

*/
        //distance = 1;
        //encoderElevator(DRIVE_SPEED, distance,3);
        //distance = -1;
        //encoderElevator(DRIVE_SPEED, distance,3);
        //distance = 1;
        //encoderInAndOut(DRIVE_SPEED,distance,2);
        //distance = -1;
        //encoderInAndOut(DRIVE_SPEED,distance,2);

        //distance = 1;
        //encoderElevator(DRIVE_SPEED, distance,6);

        //go left
        //power was 0.3 changed to 1.0
        x = 1.0;
        y = 0.0;
        turn = 0.0;
        pullIn = false;
        pullOut = false;
        boolean isRightTurn = false;

        Movement third = new Movement(x,y,turn,pullIn,pullOut, isRightTurn);

        runtime.reset();
        while (runtime.seconds() < 1) {
            fL.setPower(third.lF_power);
            fR.setPower(third.rF_power);
            bL.setPower(third.lB_power);
            bR.setPower(third.rB_power);
        }

        distance = 48;
        //move forward
        //go straight ahead
            encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 10);


        distance = 1;
        encoderInAndOut(DRIVE_SPEED,distance,6);

        distance = -2;
        encoderElevator(DRIVE_SPEED, distance,6);

        distance = -50;
        //move backward
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 10);

        distance = 2;
        encoderElevator(DRIVE_SPEED, distance,6);

        distance = -1;
        encoderInAndOut(DRIVE_SPEED,distance,6);


        distance = 2;
        //move forward
        //go straight ahead
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 2);

        distance = -4;
        //move backward
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 2);

        /*
        //go left
        x = 1.0;
        y = 0.0;
        turn = 0.0;
        power = 0.3;
        pullIn = false;
        pullOut = false;

        Movement second = new Movement(x,y,turn,power,pullIn,pullOut);

        runtime.reset();
        while (runtime.seconds() < 3) {
            fL.setPower(second.lF_power);
            fR.setPower(second.rF_power);
            bL.setPower(second.lB_power);
            bR.setPower(second.rB_power);
        }

        distance = -10;
        //move backward
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 3);

         */

        /*
        //turn left
        x = 0.0;
        y = 0.0;
        turn = 0.5;
        power = 0.3;
        pullIn = false;
        pullOut = false;

        Movement tenth = new Movement(x,y,turn,power,pullIn,pullOut);

        runtime.reset();
        while (runtime.seconds() < 2) {
            fL.setPower(tenth.lF_power);
            fR.setPower(tenth.rF_power);
            bL.setPower(tenth.lB_power);
            bR.setPower(tenth.rB_power);
        }
        */

        /*
        //go left
        x = 1.0;
        y = 0.0;
        turn = 0.0;
        power = 0.3;
        pullIn = false;
        pullOut = false;

        Movement eleventh = new Movement(x,y,turn,power,pullIn,pullOut);

        runtime.reset();
        while (runtime.seconds() < 2.5) {
            fL.setPower(eleventh.lF_power);
            fR.setPower(eleventh.rF_power);
            bL.setPower(eleventh.lB_power);
            bR.setPower(eleventh.rB_power);
        }

        
         */
        /*
        distance = 2;
        //move forward
        //go straight ahead
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 1);

        //go left
        x = 1.0;
        y = 0.0;
        turn = 0.0;
        power = 0.3;
        pullIn = false;
        pullOut = false;

        second = new Movement(x,y,turn,power,pullIn,pullOut);

        runtime.reset();
        while (runtime.seconds() < 1) {
            fL.setPower(second.lF_power);
            fR.setPower(second.rF_power);
            bL.setPower(second.lB_power);
            bR.setPower(second.rB_power);
        }
        */
        /*
        distance = 36;
        //move forward
        //go straight ahead
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 10);
        //}

        //go right
        x = -1.0;
        y = 0.0;
        turn = 0.0;
        power = 0.3;
        pullIn = false;
        pullOut = false;

        Movement third = new Movement(x,y,turn,power,pullIn,pullOut);

        runtime.reset();
        while (runtime.seconds() < 5) {
            fL.setPower(third.lF_power);
            fR.setPower(third.rF_power);
            bL.setPower(third.lB_power);
            bR.setPower(third.rB_power);
        }

        //back up
        x = 0.0;
        y = -1.0;
        turn = 0.0;
        power = 0.3;
        pullIn = false;
        pullOut = false;

        Movement fourth = new Movement(x,y,turn,power,pullIn,pullOut);

        runtime.reset();
        while (runtime.seconds() < 5) {
            fL.setPower(fourth.lF_power);
            fR.setPower(fourth.rF_power);
            bL.setPower(fourth.lB_power);
            bR.setPower(fourth.rB_power);
        }

        //go forward
        x = 0.0;
        y = 1.0;
        turn = 0.0;
        power = 0.3;
        pullIn = false;
        pullOut = false;

        Movement fifth = new Movement(x,y,turn,power,pullIn,pullOut);

        runtime.reset();
        while (runtime.seconds() < 0.5) {
            fL.setPower(fifth.lF_power);
            fR.setPower(fifth.rF_power);
            bL.setPower(fifth.lB_power);
            bR.setPower(fifth.rB_power);
        }

        //go left
        x = -1.0;
        y = 0.0;
        turn = 0.0;
        power = 0.3;
        pullIn = false;
        pullOut = false;

        Movement sixth = new Movement(x,y,turn,power,pullIn,pullOut);

        runtime.reset();
        while (runtime.seconds() < 3) {
            fL.setPower(sixth.lF_power);
            fR.setPower(sixth.rF_power);
            bL.setPower(sixth.lB_power);
            bR.setPower(sixth.rB_power);
        }

*/


    }

    public void encoderElevator(double speed,
                                double Inches1, double timeoutS){
        int new_target;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            new_target = elevator.getCurrentPosition() + (int) (Inches1 * COUNTS_PER_INCH_GOBUILDA);

            elevator.setTargetPosition(new_target);


            // Turn On RUN_TO_POSITION
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            elevator.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (elevator.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", new_target);
                telemetry.addData("Path2", "Running at %7d",
                       elevator.getCurrentPosition()
                       );
                telemetry.update();
            }

            // Stop all motion;
            elevator.setPower(0);


            // Turn off RUN_TO_POSITION
            elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        }
    }

    public void encoderInAndOut(double speed,
                                double Inches1, double timeoutS){
        int new_target;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            new_target = inAndOut.getCurrentPosition() + (int) (Inches1 * COUNTS_PER_INCH);

            inAndOut.setTargetPosition(new_target);


            // Turn On RUN_TO_POSITION
            inAndOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            inAndOut.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (inAndOut.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", new_target);
                telemetry.addData("Path2", "Running at %7d",
                        inAndOut.getCurrentPosition()
                );
                telemetry.update();
            }

            // Stop all motion;
            inAndOut.setPower(0);


            // Turn off RUN_TO_POSITION
            inAndOut.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        }
    }

    public void encoderDrive(double speed,
                             double leftInches1, double leftInches2, double rightInches1, double rightInches2,
                             double timeoutS) {

        int new_tLeftTarget;
        int new_tRightTarget;
        int new_bLeftTarget;
        int new_bRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            new_tLeftTarget = fL.getCurrentPosition() + (int) (leftInches1 * COUNTS_PER_INCH);
            new_tRightTarget = fR.getCurrentPosition() + (int) (rightInches1 * COUNTS_PER_INCH);
            new_bLeftTarget = bL.getCurrentPosition() + (int) (leftInches2 * COUNTS_PER_INCH);
            new_bRightTarget = bR.getCurrentPosition() + (int) (rightInches2 * COUNTS_PER_INCH);

            fL.setTargetPosition(new_tLeftTarget);
            fR.setTargetPosition(new_tRightTarget);
            bL.setTargetPosition(new_bLeftTarget);
            bR.setTargetPosition(new_bRightTarget);

            // Turn On RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fL.setPower(speed);
            fR.setPower(speed);
            bL.setPower(speed);
            bR.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fL.isBusy() && fR.isBusy() && bL.isBusy() && bR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", new_tLeftTarget, new_tRightTarget, new_bLeftTarget, new_bRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        fL.getCurrentPosition(),
                        fR.getCurrentPosition(),
                        bL.getCurrentPosition(),
                        bR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            fL.setPower(0);
            fR.setPower(0);
            bL.setPower(0);
            bR.setPower(0);

            // Turn off RUN_TO_POSITION
            fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            }

        }
}

