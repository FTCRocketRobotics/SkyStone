package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;





    @TeleOp


    public class DosaOpMode extends LinearOpMode {

        private DcMotor fL;
        private DcMotor fR;
        private DcMotor bL;
        private DcMotor bR;

    @Override

        public void runOpMode() {

            telemetry.addData("Status", "Go!");
            telemetry.update();
            bL = hardwareMap.get(DcMotor.class, "bL");
            bR = hardwareMap.get(DcMotor.class, "bR");
            fL = hardwareMap.get(DcMotor.class, "fL");
            fR = hardwareMap.get(DcMotor.class, "fR");

            waitForStart();

            fL.setDirection(DcMotor.Direction.FORWARD);
            fR.setDirection(DcMotor.Direction.REVERSE);
            bL.setDirection(DcMotor.Direction.FORWARD);
            bR.setDirection(DcMotor.Direction.REVERSE);

            while (opModeIsActive());

            {
                double powerfL;
                double powerfR;
                double powerbL;
                double powerbR;

                double drive = -gamepad1.left_stick_y;
                powerfL= Range.clip(drive,-1.0,1.0);
                powerfR= Range.clip(drive,-1.0,1.0);
                powerbL= Range.clip(drive,-1.0,1.0);
                powerbR= Range.clip(drive,-1.0,1.0);


                fR.setPower(powerfR);
                fL.setPower(powerfL);
                bR.setPower(powerfR);
                bL.setPower(powerfL);


                /**bL.setPower(1.0);
                 bR.setPower(1.0);
                 fL.setPower(1.0);
                 fR.setPower(1.0); */

                telemetry.addData("Target Power", powerfL);
                telemetry.addData("Motor Power",fL.getPower());
                telemetry.addData("status","running");
                telemetry.update();
            }






        }

    }


