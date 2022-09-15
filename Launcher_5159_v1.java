 package org.firstinspires.ftc.teamcode;

 import android.graphics.Color;
 import android.util.Log;


 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Gamepad;
 import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
 import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
 import com.qualcomm.robotcore.hardware.NormalizedRGBA;
 import com.qualcomm.robotcore.hardware.SwitchableLight;

 import org.firstinspires.ftc.robotcore.external.ClassFactory;
 import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
 import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
 import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;



 import java.util.List;


 public class Launcher_5159_v1 {

     public DcMotor mtrCarousel = null;
     public DcMotor mtrShooter = null;
     public DcMotor mtrIntake = null;
     public DcMotor mtrElev = null;

     // Servos
     public CRServo srvoDropper = null;
     public Servo srvoLeftPitstop = null;
     public Servo srvoRightPitstop = null;

     // Motor constants
     double dPwrIntake = 0.7;
     double dPwrCarousel = -0.9;
     double dPwrShooterPowerShot = -0.61;
     double dPwrShooterMidGoal = -0.68;
     double dPwrShooterTopGoalFirst = -0.641;
     double dPwrShooterTopGoal = -0.643;
     double dPwrElevUp = 0.4;
     double dPwrElevDown = -0.2;

     // Servo constants
     double dPwrDropper = 1.0;
     double dPitstopUpPos = 1.0;
     double dPitstopDownPos = 0.0;

     // Calc RPM Constants
     double previousShooterPosition = 0;
     double previousTime = 0;
     double RPM = 0;
     double shooterPosition = 0;
     double currentTime = 0;

     public Launcher_5159_v1() {

     }

     public void initLauncher(OpMode opMode, String strElevName,
                              String strIntakeName, String strCarouselName, String strShooterName,
                              String strDropperName, String strLeftPitstopName, String strRightPitstopName) {
         // Elevator
         mtrElev = opMode.hardwareMap.get(DcMotor.class, strElevName);
         mtrElev.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
         // Intake
         mtrIntake = opMode.hardwareMap.get(DcMotor.class, strIntakeName);
         mtrIntake.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
         // Carousel
         mtrCarousel = opMode.hardwareMap.get(DcMotor.class, strCarouselName);
         mtrCarousel.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise

         // Shooter
         mtrShooter = opMode.hardwareMap.get(DcMotor.class, strShooterName);
         mtrShooter.setDirection(DcMotor.Direction.FORWARD); // Positive input rotates counter clockwise
         mtrShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         // Dropper
         srvoDropper = opMode.hardwareMap.get(CRServo.class, strDropperName);
         srvoDropper.setDirection(CRServo.Direction.FORWARD);

         // Left pitstop
         srvoLeftPitstop = opMode.hardwareMap.get(Servo.class, strLeftPitstopName);
         srvoLeftPitstop.setDirection(Servo.Direction.FORWARD);

         // Right pitstop
         srvoRightPitstop = opMode.hardwareMap.get(Servo.class, strRightPitstopName);
         srvoRightPitstop.setDirection(Servo.Direction.FORWARD);

         mtrElev.setPower(0);
         mtrIntake.setPower(0);
         mtrCarousel.setPower(0);
         mtrShooter.setPower(0);

         // Set servo positions?

     }

     // Autonomous Elevator
     public void autonMoveElev(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
         long lMarkMilliS;
         lMarkMilliS = System.currentTimeMillis();
         // Elevator Motor (encoder)
         mtrElev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrElev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         mtrElev.setPower(speed);

         lMarkMilliS = System.currentTimeMillis() + timeout;
         while ((System.currentTimeMillis() < lMarkMilliS) && opMode.opModeIsActive()
                 && Math.abs(mtrElev.getCurrentPosition()) < distance
         ) {
             opMode.telemetry.addData(msg + " Elev pos:", "%d",
                     mtrElev.getCurrentPosition());
             opMode.telemetry.update();
         }
         mtrElev.setPower(0);
     }

     // Autonomous Intake
     public void autonMoveIntake(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
         long lMarkMilliS;
         lMarkMilliS = System.currentTimeMillis();
         //Intake Motor (encoder)
         mtrIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         mtrIntake.setPower(speed);

         lMarkMilliS = System.currentTimeMillis() + timeout;
         while ((System.currentTimeMillis() < lMarkMilliS) && opMode.opModeIsActive()
                 && Math.abs(mtrIntake.getCurrentPosition()) < distance
         ) {
             opMode.telemetry.addData(msg + " Intake pos:", "%d",
                     mtrIntake.getCurrentPosition());
             opMode.telemetry.update();
         }
         mtrIntake.setPower(0);
     }

     /* ----------------------- */

     // Autonomous Carousel
     public void autonMoveCarousel(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
         long lMarkMilliS;
         lMarkMilliS = System.currentTimeMillis();
         //Shooter Motor (encoder)
         //mtrCarousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         //mtrCarousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         mtrCarousel.setPower(speed);

         lMarkMilliS = System.currentTimeMillis() + timeout;
         while ((System.currentTimeMillis() < lMarkMilliS) && opMode.opModeIsActive()
                 && Math.abs(mtrCarousel.getCurrentPosition()) < distance
         ) {
             opMode.telemetry.addData(msg + " Intake pos:", "%d",
                     mtrCarousel.getCurrentPosition());
             opMode.telemetry.update();
         }
         mtrCarousel.setPower(0);
     }

     // Autonomous Shooter
     public void autonMoveShooter(LinearOpMode opMode, String msg, double speed, int distance, int timeout) {
         long lMarkMilliS;
         lMarkMilliS = System.currentTimeMillis();
         //Shooter Motor (encoder)
         mtrShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         mtrShooter.setPower(speed);

         lMarkMilliS = System.currentTimeMillis() + timeout;
         while ((System.currentTimeMillis() < lMarkMilliS) && opMode.opModeIsActive()
                 && Math.abs(mtrShooter.getCurrentPosition()) < distance
         ) {
             opMode.telemetry.addData(msg + " Intake pos:", "%d",
                     mtrShooter.getCurrentPosition());
             opMode.telemetry.update();
         }
         mtrShooter.setPower(0);
     }

     // Autonomous Dropper
     public void autonMoveDropper(LinearOpMode opMode, String msg, double power, int timeout) {
         long lMarkMilliS;
         lMarkMilliS = System.currentTimeMillis() + timeout;
         srvoDropper.setPower(power);
         while ((System.currentTimeMillis() < lMarkMilliS) && opMode.opModeIsActive()) {
             opMode.telemetry.addData(msg + " Dropper pos:", "%.2f",
                     srvoDropper.getPower());
             opMode.telemetry.update();
         }
         srvoDropper.setPower(0);
     }

     // Autonomous Pitstop
     public void autonMovePitstop(LinearOpMode opMode, String msg, boolean up, int timeout) {
         long lMarkMilliS;
         lMarkMilliS = System.currentTimeMillis() + timeout;
         if (up == true) {
             srvoLeftPitstop.setPosition(dPitstopUpPos);
             srvoRightPitstop.setPosition(dPitstopUpPos);
         } else {
             srvoLeftPitstop.setPosition(dPitstopDownPos);
             srvoRightPitstop.setPosition(dPitstopDownPos);
         }
         while ((System.currentTimeMillis() < lMarkMilliS) && opMode.opModeIsActive()) {
             opMode.telemetry.addData(msg + " Pitstop pos:", "%.2f",
                     srvoLeftPitstop.getPosition());
             opMode.telemetry.update();
         }
     }


     // ---------------------------------------------------------------------------------------------------------------------//
/*
    // Shoot Rings (Not used)
     public void shootRings(LinearOpMode opMode) {
         long lMarkMilliS;
         lMarkMilliS=System.currentTimeMillis();

         //Shooter Motor (encoder)
         mtrShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         mtrShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


         mtrShooter.setPower(-0.64);
         lMarkMilliS=System.currentTimeMillis() + 2750;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
         ) { }

         mtrCarousel.setPower(dPwrCarousel);

         lMarkMilliS=System.currentTimeMillis() + 600;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
         ) { }

         mtrCarousel.setPower(0);
         mtrShooter.setPower(-0.644);

         lMarkMilliS=System.currentTimeMillis() + 1500;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
         ) { }

         mtrCarousel.setPower(dPwrCarousel);
         lMarkMilliS=System.currentTimeMillis() + 750;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
         ) { }
         mtrIntake.setPower(dPwrIntake);
         mtrShooter.setPower(-0.64);


         lMarkMilliS=System.currentTimeMillis() + 3500;
         while((System.currentTimeMillis() < lMarkMilliS)  && opMode.opModeIsActive()
         ) { }
         mtrShooter.setPower(0);
         mtrCarousel.setPower(0);
         mtrIntake.setPower(0);
     }
     
 */
     // Teleop Elevator
     void telMoveElev(Gamepad gamepad2) {
         //            Y
         //        X       B
         //            A
         //

         // Manual control
         if (gamepad2.left_trigger > (0.67)) {
             mtrElev.setPower(dPwrElevDown);
         } else if (gamepad2.left_bumper) {
             mtrElev.setPower(dPwrElevUp);
         } else {
             mtrElev.setPower(0.0);
         }
     }

     // Teleop Intake
     void telMoveIntake(Gamepad gamepad2) {
         // Manual control
         if (gamepad2.dpad_up) {
             mtrIntake.setPower(dPwrIntake);
         } else if (gamepad2.dpad_down) {
             mtrIntake.setPower(-dPwrIntake);
         } else {
             mtrIntake.setPower(0.0);
         }
     }

     // Teleop Carousel
     void telMoveCarousel(Gamepad gamepad2) {
         //manual control
         if (gamepad2.a) {
             mtrCarousel.setPower(-dPwrCarousel);
         } else if (gamepad2.y) {
             mtrCarousel.setPower(dPwrCarousel);
         } else {
             mtrCarousel.setPower(0.0);
         }
     }

     // Teleop Shooter
     void telMoveShooter(Gamepad gamepad2) {
         //manual control
         if (gamepad2.right_bumper) {
             mtrShooter.setPower(dPwrShooterPowerShot);
         } else if (gamepad2.right_trigger > (0.67)) {
             mtrShooter.setPower(dPwrShooterTopGoal);
         } else {
             mtrShooter.setPower(0.0);
         }
     }

     // Teleop Dropper
     void telMoveDropper(Gamepad gamepad2) {
         // slew the servo, according to the rampUp (direction) variable.
         if (gamepad2.dpad_left) {
             srvoDropper.setPower(-dPwrDropper);
         } else if (gamepad2.dpad_right) {
             srvoDropper.setPower(dPwrDropper);
         } else {
             srvoDropper.setPower(0);
         }
     }

     // Teleop Pitstop
     void telMovePitstop(Gamepad gamepad2) {
         // slew the servo, according to the rampUp (direction) variable.
         if (gamepad2.dpad_left) {
             srvoLeftPitstop.setPosition(dPitstopUpPos);
             srvoRightPitstop.setPosition(dPitstopUpPos);
         } else if (gamepad2.dpad_right) {
             srvoLeftPitstop.setPosition(dPitstopDownPos);
             srvoRightPitstop.setPosition(dPitstopDownPos);
         } else {

         }
     }

     //calculate the RPM of the shooter

    /*
    public void calcRPM(OpMode opMode) {
         long lMarkMilliS;


        shooterPosition = mtrShooter.getCurrentPosition();
        currentTime = System.currentTimeMillis();
        if((currentTime-previousTime)<100) return;
        RPM = (shooterPosition-previousShooterPosition)/(currentTime-previousTime);

        previousShooterPosition = shooterPosition;
        previousTime = currentTime;

        opMode.telemetry.addData( "Shooter rate:", "%f",
                RPM);
        opMode.telemetry.update();
    }

     */
 }
 