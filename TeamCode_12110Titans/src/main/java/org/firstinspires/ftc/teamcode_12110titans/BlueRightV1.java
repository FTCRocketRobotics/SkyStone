package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueRightV1 extends LinearOpMode {

    private DcMotor fL;
    private DcMotor fR;
    private DcMotor bL;
    private DcMotor bR;

    static final double COUNTS_PER_MOTOR_REV_GOBUILDA = 383.06;    // // eg: GoBuilda Motor Encoder
    static final double COUNTS_PER_MOTOR_REV_ANDYMARK = 1120.00;    // // eg: AndyMark neverest 40
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_ANDYMARK * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //changed turn and power 0.5 to 1.0
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

        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                fL.getCurrentPosition(),
                fR.getCurrentPosition(),
                bL.getCurrentPosition(),
                bR.getCurrentPosition());
        telemetry.update();

        waitForStart();

        int distance = 24;
        //changes power 0.8 to 1.0
        //while (!isStopRequested()) {
        double x = -1.0;
        double y = 0.0;
        double turn = 0.0;
        double power = 1.0;
        boolean pullIn = false;
        boolean pullOut = false;

        Movement first = new Movement(x,y,turn,power,pullIn,pullOut);

        runtime.reset();
        while (runtime.seconds() < 0.2) {
            fL.setPower(first.lF_power*0.3);
            fR.setPower(first.rF_power*0.3);
            bL.setPower(first.lB_power*0.3);
            bR.setPower(first.rB_power*0.3);
        }

            //move forward
            //go straight ahead

            encoderDrive(DRIVE_SPEED, distance, distance, distance, distance, 10);
        //}


    }

    public void encoderDrive(double speed,
                             double leftInches1, double leftInches2, double rightInches1, double rightInches2,
                             double timeoutS) {

        int new_tLeftTarget;
        int new_tRightTarget;
        int new_bLeftTarget;
        int new_bRightTarget;

        // Ensure that the OpMode is still active
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

