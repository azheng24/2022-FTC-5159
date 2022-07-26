package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;



public class ArcherFish {
  
    private DcMotor mtrShooter;
    private Servo srvoTrigger;

    private Double dMtrPwr=.99d;
    
    private static double TRIGGER_COCK = .55d;
    private static double TRIGGER_PULL = .25d;

    // todo: write your code here
    public void operate (OpMode opmode, Gamepad gamepad) {
        if(gamepad.left_trigger>.5) {
            srvoTrigger.setPosition(TRIGGER_PULL);
        }
        else {
            srvoTrigger.setPosition(TRIGGER_COCK);
            
        }
        //set shooter power
        if(gamepad.dpad_down){
            mtrShooter.setPower(0);
        } 

        else if(gamepad.dpad_up) {
            mtrShooter.setPower(.9d);
        }
    }
    public  void initialize(OpMode opMode) {
        opMode.telemetry.addData("ArcherFish","Initializing...");
        
        mtrShooter = opMode.hardwareMap.get(DcMotor.class, "mtrShooter");
        opMode.telemetry.addData("ArcherFish","    Shooter Motor Off");
        mtrShooter.setPower(0d);
        
        srvoTrigger = opMode.hardwareMap.get(Servo.class, "srvoTrigger");
        opMode.telemetry.addData("ArcherFish","    Trigger Cock");
        srvoTrigger.setPosition(TRIGGER_COCK);
        
    }
    public void shutdown(OpMode opMode) {
        mtrShooter.setPower(0d);
        srvoTrigger.setPosition(TRIGGER_COCK);
        
        
    }
}
