/*
Copyright 2021 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *com.qualcomm
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */

public class Monkey  {
    /* Declare OpMode members. */
    private DcMotor mtrMonkeyLeftArm, mtrMonkeyRightArm;
   
    //Monkey trunk motor constants
    private static Double MONKEY_PULLUP_PWR=.99d, MONKEY_DOWN_PWR=-.66d;
    private static Double STICK_DEAD_ZONE=.5;
    
    
    
    private final int CAPSTATE_IDLE=0;
    private final int CAPSTATE_BITING = 1;
    private final int CAPSTATE_RAISING_NECK= 2;
    private final int CAPSTATE_EXTENDING_NECK=3;
    private final int CAP_NECK_EXTEND_POS=2500;
    private long glCapTimeStamp=0;
    private boolean gbCapStarted=false;
    private int gnCapState = CAPSTATE_IDLE;
    
    public void init(OpMode opMode) {
  
        opMode.telemetry.addData("Monkey", "Initializing...");
        opMode.telemetry.addData("Monkey", "    Both arms must be down");
        opMode.telemetry.addData("Monkey", "    If not, Hit Stop, then re-Init");
        
        //Monkey Motor
        mtrMonkeyLeftArm = opMode.hardwareMap.get(DcMotor.class, "mtrMonkeyLeftArm");
        mtrMonkeyLeftArm.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrMonkeyRightArm = opMode.hardwareMap.get(DcMotor.class, "mtrMonkeyRightArm");
        mtrMonkeyRightArm.setDirection(DcMotorSimple.Direction.FORWARD);
        //motor_ET.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        mtrMonkeyLeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrMonkeyLeftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        mtrMonkeyLeftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrMonkeyRightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrMonkeyRightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        mtrMonkeyRightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        
        opMode.telemetry.addData("Monkey", "    Initialized");
        

    }

    public void operate(OpMode opMode) {
        opMode.telemetry.addData("Monkey","Left Pos:%d Pwr:%.2f",
           mtrMonkeyLeftArm.getCurrentPosition(),mtrMonkeyLeftArm.getPower());
      
        opMode.telemetry.addData("Monkey","Right Pos:%d Pwr:%.2f",
           mtrMonkeyRightArm.getCurrentPosition(),mtrMonkeyRightArm.getPower());

         //elevator motor 
        if(opMode.gamepad2.a) {
            mtrMonkeyLeftArm.setPower(MONKEY_PULLUP_PWR);
            mtrMonkeyRightArm.setPower(MONKEY_PULLUP_PWR);
            return;
        }
        if(opMode.gamepad2.b) {
            mtrMonkeyLeftArm.setPower(MONKEY_DOWN_PWR);
            mtrMonkeyRightArm.setPower(MONKEY_DOWN_PWR);
            return;
        }
        mtrMonkeyLeftArm.setPower(0);
        mtrMonkeyRightArm.setPower(0);
        
        
        //teleopCap(false);
        // if(opMode.gamepad2.y) {
        //     srvoMonkeyMouth.setPosition(GIRAFFE_BITE_BLOCK);
        //     return;
        // }
        

    }
    /*
    public void autonExtendNeckLOp(LinearOpMode linopMode, int nExtendPos) {
        int nNeckCurrPos=mtrMonkey.getCurrentPosition();
        double dPwr=0d;
        //check if need to extend or contract
        if(nExtendPos>nNeckCurrPos) {
            //need to extend
            dPwr=GIRAFFE_EXTEND_PWR;
            while(nExtendPos>mtrMonkey.getCurrentPosition()){
                mtrMonkey.setPower(GIRAFFE_EXTEND_PWR);
            }
            mtrMonkey.setPower(0d);
        }
        else if (nExtendPos<nNeckCurrPos) {
            dPwr=GIRAFFE_EXTEND_PWR;
            while(nExtendPos<mtrMonkey.getCurrentPosition()){
                mtrMonkey.setPower(-GIRAFFE_EXTEND_PWR);
            }
            mtrMonkey.setPower(0d);
        }
    }
    
    public void autonNeckDownLOp(LinearOpMode linopMode) {
        srvoMonkeyNeck.setPosition(GIRAFFE_NECK_DOWN);
    }
    
    public void autonNeckUpLOp(LinearOpMode linopMode) {
        srvoMonkeyNeck.setPosition(GIRAFFE_NECK_UP);
    }

    public void autonOpenMouthLOp(LinearOpMode linopMode) {
        srvoMonkeyMouth.setPosition(GIRAFFE_MOUTH_OPEN);
    }
    
    public void autonCloseMouthLOp(LinearOpMode linopMode) {
        srvoMonkeyMouth.setPosition(GIRAFFE_MOUTH_CLOSED);
    }
    private int teleopCap(boolean bButtonPressed) {
        //if got here, the cap button was pressed
        //check if this is the first time pressed
        long lTimeStamp;
        if(bButtonPressed) {
            //starting capping
            glCapTimeStamp=System.currentTimeMillis();
            gnCapState=CAPSTATE_BITING;
            srvoMonkeyMouth.setPosition(GIRAFFE_BITE_ELEMENT);
        }
            
        
        
        switch(gnCapState){
            case CAPSTATE_IDLE:
                return CAPSTATE_IDLE;
            case CAPSTATE_BITING:  //biting
                lTimeStamp=System.currentTimeMillis();
                if((lTimeStamp-glCapTimeStamp)>300) {//300 ms to bite
                   glCapTimeStamp=lTimeStamp;
                   gnCapState=CAPSTATE_RAISING_NECK;
                   return CAPSTATE_RAISING_NECK;
               
                }
                return CAPSTATE_BITING;
            case CAPSTATE_RAISING_NECK: //raising neck
                srvoMonkeyNeck.setPosition(GIRAFFE_NECK_DOWN); //makes neck level to mat
                gnCapState=CAPSTATE_EXTENDING_NECK;
                return CAPSTATE_EXTENDING_NECK;
            case CAPSTATE_EXTENDING_NECK:
                int nNeckCurrPos=mtrMonkey.getCurrentPosition();
                double dPwr=0d;
                //check if need to extend or contract
                if(CAP_NECK_EXTEND_POS>nNeckCurrPos) {
                    //need to extend
                    dPwr=GIRAFFE_EXTEND_PWR;
                    mtrMonkey.setPower(GIRAFFE_EXTEND_PWR);
                    return (CAPSTATE_EXTENDING_NECK);
                    
                }
                //done extending
                mtrMonkey.setPower(0d);
                
                gnCapState=CAPSTATE_IDLE;
                gbCapStarted=false;
                return (gnCapState);
            default:
                return (gnCapState); 
        }
    
        
    }
    */
}
