package org.firstinspires.ftc.teamcode_12110titans;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PizzaOpMode extends LinearOpMode {
   private Gyroscope waterMelonShark;
   private DcMotor Test;
   private DigitalChannel spongeBobSandwich;
   private DistanceSensor pokemonBurgers;
   private Servo servoTest;

   @Override
    public void runOpMode() {
       waterMelonShark=hardwareMap.get(Gyroscope.class,"waterMelonshark");
       Test=hardwareMap.get(DcMotor.class,"Test");
       spongeBobSandwich=hardwareMap.get(DigitalChannel.class,"spongeBobSandwich");
       pokemonBurgers=hardwareMap.get(DistanceSensor.class,"pokemonBurgers");
       servoTest=hardwareMap.get(Servo.class,"servoTest");

       telemetry.addData("Status","Terminate");
       telemetry.update();
       //Wait for the game to start(driver presses PLAY)
       waitForStart();

       //run until the end of the match(driver presses STOP)
           while(opModeIsActive()){
               telemetry.addData("Status","Running");
               telemetry.update();
           }


   }
}
