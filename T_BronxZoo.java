  /*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


  @TeleOp(name="T_BronxZoo", group="Demonstration")
  //@Disabled
  public class T_BronxZoo extends OpMode {

      //private  boolean gbVisionProb=false,gbNavXProb=false;
        /* Declare OpMode members. */
      private Crab_v1 crab = new Crab_v1();   // Use Omni-Directional drive system
      private ArcherFish archerfish = new ArcherFish(); 

      @Override
      public void init() {
          crab.initialize(this, crab.CHASSIS_LEFT_FWD,"mtrLeftFront",
                  "mtrLeftBack", "mtrRightFront","mtrRightBack",
                  "navx",25, .85);
          crab.chassisDontUseLeftTrigBmpr(true);
          crab.resetTeleop(false);
          archerfish.initialize(this);
          // Wait for the game to start (driver presses PLAY)
         // Prompt User
          telemetry.addData("Press Start", " >");
          telemetry.update();
      }

      @Override
      public void stop() {

          crab.closeGraceful(this);
          archerfish.shutdown(this);
          telemetry.addData("[]", "Shutting Down. Bye!");
          telemetry.update();
      }

      /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
      */
      @Override
      public void init_loop() {

      }

      /*
       * This method will be called repeatedly in a loop
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
      */
      @Override
      public void loop() {

          // run until the end of the match (driver presses STOP)
          crab.operate(this,gamepad1);
          archerfish.operate(this,gamepad2);

          telemetry.update();

      }
      private void init_hardware() {


      }
  }
