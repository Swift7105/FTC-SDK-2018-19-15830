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

package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.s
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 *
 *
 * +++++++++++++++++++++++++
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareRobot
{
    /* Public OpMode members. */
    public DcMotor  LeftFrontDrive   = null;
    public DcMotor  RightFrontDrive  = null;
    public DcMotor  LeftBackDrive     = null;
    public DcMotor  RightBackDrive = null;
    public DcMotor Arm = null;
    public DcMotor Lift = null;



  //  public Servo Door = null;
  //  public CRServo Intake = null;
 //   public CRServo Slider = null;
  //  public CRServo arm2 = null;




    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LeftFrontDrive  = hwMap.get(DcMotor.class, "Left_Front_Drive");
        LeftBackDrive  = hwMap.get(DcMotor.class, "Left_Back_Drive");
        RightFrontDrive  = hwMap.get(DcMotor.class, "Right_Front_Drive");
        RightBackDrive  = hwMap.get(DcMotor.class, "Right_Back_Drive");
        Arm = hwMap.get(DcMotor.class, "Arm");
        Lift = hwMap.get(DcMotor.class, "Lift");


        //Door = hwMap.get(Servo.class,"Door");
       // Slider = hwMap.get(CRServo.class, "Slider");
       // arm2 = hwMap.get(CRServo.class, "arm2");
       // Intake = hwMap.get(CRServo.class,"Intake");


        // Set all motors to zero power
        LeftFrontDrive.setPower(0);
        RightFrontDrive.setPower(0);
        RightBackDrive.setPower(0);
        LeftBackDrive.setPower(0);
        Arm.setPower(0);
        Lift.setPower(0);
        /*
        Arm.setPower(0);
        Lift.setPower(0);
        Intakearm.setPower(0);
        */

        //Door.setPosition(0);
        //Slider.setPower(0);
        //arm2.setPower(0);



        LeftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        LeftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        RightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        /*
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
*/



        // Set all motors to run withoutcoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LeftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

      /*
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/

    }

}

