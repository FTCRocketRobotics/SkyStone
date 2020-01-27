package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueAuto extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV_GOBUILDA = 383.06; //eg: GoBuilda Motor Encoder
    static final double COUNTS_PER_MOTOR_REV_ANDYMARK = 1120.00; //eg: AndyMark neverest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_ANDYMARK * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_INCH_GOBUILDA =
            (COUNTS_PER_MOTOR_REV_GOBUILDA * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = .5;
    static final double TURN_SPEED = .5;
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;
    private DcMotor elevator;
    private DcMotor spinny;
    private DcMotor inAndOut;
    //distance
    private int distance = 0;
    //limit switch
    private DigitalChannel jackieChan;
    private ElapsedTime runtime = new ElapsedTime();

    private void startRunOpMode() {

        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");

        bL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        spinny = hardwareMap.get(DcMotor.class, "spinny");
        inAndOut = hardwareMap.get(DcMotor.class, "inAndOut");

        jackieChan = hardwareMap.get(DigitalChannel.class, "jackieChan");

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    }


    private void moveSides(double x, double y, double turn,
                           boolean pullIn, boolean pullOut) {

        Movement mSides = new Movement(x, y, turn, pullIn, pullOut);
        runtime.reset();
        while (runtime.seconds() < 1) {
            fL.setPower(mSides.lF_power);
            fR.setPower(mSides.rF_power);
            bL.setPower(mSides.lB_power);
            bR.setPower(mSides.rB_power);
        }
    }

    /**
     *
     */
    @Override
    public void runOpMode() {

        // Initialize pre runOpMode values
        startRunOpMode();

        // Robot is turning Left
        Movement mSides = new Movement(1.0, 0, 0, 0, false, false);
        runtime.reset();
        while (runtime.seconds() < 1) {
            fL.setPower(mSides.lF_power);
            fR.setPower(mSides.rF_power);
            bL.setPower(mSides.lB_power);
            bR.setPower(mSides.rB_power);
        }
        //moveSides(1.0,0.0,0.0,false, false);

        // Robot is moving forward
        distance = 48; // 48
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 10);

        distance = 1;
        //encoderInAndOut(DRIVE_SPEED,distance,6);

        distance = -2;
        //encoderElevator(DRIVE_SPEED, distance,6);

        distance = -45;  // 50
        //move backward
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 10);

        //sleep(1000);

        distance = 2;
        //encoderElevator(DRIVE_SPEED, distance,6);

        distance = -1;
        //encoderInAndOut(DRIVE_SPEED,distance,6);

        // Robot is turning right
        //moveSides(-.6, 0.0, 0.0, 0.2, false, false);

        //sleep(2000);

        moveSides(0.0, 0.0, -.17, false, false);

        //move forward
        //go straight ahead
        distance = 63; // 48
        encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 10);

    }

    public void encoderElevator(double speed,
                                double Inches1, double timeoutS) {
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

            /* keep looping while we are still active, and there is time left,
             and both motors are running. */

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
                                double Inches1, double timeoutS) {
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
//Written by: Vibhav Javali, helped by Chinmayi Ananatha and Anantha Javali.