/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode_13749atlas;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Atlas TeleOp OpMode", group="Iterative Opmode")

public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor blDrive = null;
    //private DcMotor frDrive = null;
    //private DcMotor flDrive =null;
    //private DcMotor brDrive =null;
    x_Drive_Base robot   = new x_Drive_Base();   // Use a xdrive hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //blDrive = hardwareMap.get(DcMotor.class, "bl_drive");
        //frDrive = hardwareMap.get(DcMotor.class, "fr_drive");
        //flDrive = hardwareMap.get(DcMotor.class, "fl_drive");
        //brDrive = hardwareMap.get(DcMotor.class, "br_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //flDrive.setDirection(DcMotor.Direction.REVERSE);
        //frDrive.setDirection(DcMotor.Direction.FORWARD);
        //blDrive.setDirection(DcMotor.Direction.REVERSE);
        //brDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //
        if (gamepad1.a == true)
        {
            if (robot.bLinearPickupMode == true)
            {
                robot.bLinearPickupMode = false;
            }
            else
            {
                robot.bLinearPickupMode = true;
            }
        }

        if (gamepad1.b == true)
        {
            if (robot.bCubicPickupMode == true)
            {
                robot.bCubicPickupMode = false;
            }
            else
            {
                robot.bCubicPickupMode = true;
            }
        }

        // Setup a variable for each drive wheel to save power level for telemetry
        double blPower;
        double frPower;
        double flPower;
        double brPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double left_stick_y = -gamepad1.left_stick_y;
        double left_stick_x = gamepad1.left_stick_x;
        double left_trigger = gamepad1.left_trigger;
        double right_trigger = gamepad1.right_trigger;

        if (robot.bLinearPickupMode)
        {
            left_stick_y *= robot.fPickupModePower;
            left_stick_x *= robot.fPickupModePower;
            left_trigger *= robot.fPickupModePower;
            right_trigger *= robot.fPickupModePower;

        }
        else if (robot.bCubicPickupMode)
        {
            left_stick_y = Math.pow(left_stick_y, 3);
            left_stick_x  = Math.pow(left_stick_x, 3);
            left_trigger = Math.pow(left_trigger, 3);
            right_trigger = Math.pow(right_trigger, 3);
        }

        flPower = Range.clip(left_stick_y - left_stick_x + left_trigger - right_trigger, -1.0, 1.0);
        frPower = Range.clip(left_stick_y + left_stick_x - left_trigger + right_trigger, -1.0, 1.0);
        blPower = Range.clip(-left_stick_y + left_stick_x + left_trigger - right_trigger, -1.0, 1.0);
        brPower = Range.clip(-left_stick_y - left_stick_x - left_trigger + right_trigger, -1.0, 1.0);

        // Send calculated power to wheels
        robot.fl.setPower(flPower);
        robot.fr.setPower(frPower);
        robot.bl.setPower(blPower);
        robot.br.setPower(brPower);

        ///////////


        double elevatorPower;
        double elevate = -gamepad1.right_stick_y;
        //            //double turn  =  gamepad1.right_stick_x;
                   elevatorPower = Range.clip( elevate, -1.0, 1.0) ;
        //            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;



        // Send calculated power to Elevator
        if ((elevatorPower <0 )&& (robot.elevatorLimitLower.getState() != true))
            robot.elevator.setPower(elevatorPower);
        else if ((elevatorPower >0 )&& (robot.elevatorLimitUpper.getState() != true))
            robot.elevator.setPower(elevatorPower);
        else
            robot.elevator.setPower(0);

        //
        double servoPosition = 0;
        if (gamepad1.left_bumper == true)
        {
            servoPosition = robot.grabber.MIN_POSITION;
            robot.grabber.setPosition(servoPosition);
        }
        else if (gamepad1.right_bumper == true)
        {
            servoPosition = robot.grabber.MAX_POSITION;
            robot.grabber.setPosition(servoPosition);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", left_stick_x, left_stick_y);
        telemetry.addData("Servo", "position (%.2f)", servoPosition);
        telemetry.addData("Servo", "current position (%.2f)", robot.grabber.getPosition());
        telemetry.addData("limitswitch upper", "on/off (%b)", robot.elevatorLimitUpper.getState());
        telemetry.addData("limitswitch lower", "on/off (%b)", robot.elevatorLimitLower.getState());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
