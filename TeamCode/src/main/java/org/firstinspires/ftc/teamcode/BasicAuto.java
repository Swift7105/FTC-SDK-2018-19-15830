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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Pushbot: BasicAuto", group = "Pushbot")
//@Disableds
public class BasicAuto extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot(); // use the class created to define a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();


        robot.Lift.setPower(.7);
        sleep(2000);
        robot.Lift.setPower(0);
        robot.LeftFrontDrive.setPower(.1);
        robot.LeftBackDrive.setPower(.1);
        robot.RightFrontDrive.setPower(.1);
        robot.RightBackDrive.setPower(.1);
        sleep(1000);
        robot.LeftFrontDrive.setPower(.3);
        robot.LeftBackDrive.setPower(.3);
        robot.RightFrontDrive.setPower(-.3);
        robot.RightBackDrive.setPower(-.3);
        sleep(500);
        robot.LeftFrontDrive.setPower(.2);
        robot.LeftBackDrive.setPower(.2);
        robot.RightFrontDrive.setPower(.2);
        robot.RightBackDrive.setPower(.2);
        sleep(2000);
        robot.LeftFrontDrive.setPower(.3);
        robot.LeftBackDrive.setPower(.3);
        robot.RightFrontDrive.setPower(-.3);
        robot.RightBackDrive.setPower(-.3);
        sleep(1000);
        robot.LeftFrontDrive.setPower(.3);
        robot.LeftBackDrive.setPower(.3);
        robot.RightFrontDrive.setPower(.3);
        robot.RightBackDrive.setPower(.3);
        sleep(1000);
        robot.LeftFrontDrive.setPower(.1);
        robot.LeftBackDrive.setPower(.1);
        robot.RightFrontDrive.setPower(-.1);
        robot.RightBackDrive.setPower(-.1);
        sleep(500);
        robot.Arm.setPower(-1);
        sleep(1000);
        robot.Arm.setPower(.1);
        sleep(1000);
        robot.LeftFrontDrive.setPower(.3);
        robot.LeftBackDrive.setPower(.3);
        robot.RightFrontDrive.setPower(.3);
        robot.RightBackDrive.setPower(.3);
        sleep(5000);
        robot.LeftFrontDrive.setPower(-.3);
        robot.LeftBackDrive.setPower(-.3);
        robot.RightFrontDrive.setPower(-.3);
        robot.RightBackDrive.setPower(-.3);
        sleep(500);
        robot.LeftFrontDrive.setPower(.2);
        robot.LeftBackDrive.setPower(.2);
        robot.RightFrontDrive.setPower(-.2);
        robot.RightBackDrive.setPower(-.2);
        sleep(1000);
        robot.LeftFrontDrive.setPower(-.9);
        robot.LeftBackDrive.setPower(-.9);
        robot.RightFrontDrive.setPower(-.9);
        robot.RightBackDrive.setPower(-.9);
        sleep(1000);
        robot.LeftFrontDrive.setPower(-.9);
        robot.LeftBackDrive.setPower(-.9);
        robot.RightFrontDrive.setPower(-.9);
        robot.RightBackDrive.setPower(-.9);
        sleep(1000);


        //robot.LeftFrontDrive.setPower(-.1);
       // robot.RightFrontDrive.setPower(-.1);
       // robot.RightBackDrive.setPower(-.1);
        //robot.LeftBackDrive.setPower(-.1);
       // sleep(5000);
        //robot.LeftFrontDrive.setPower(.7);
        //robot.RightFrontDrive.setPower(-.7);
        //robot.LeftBackDrive.setPower(.7);
        //robot.RightBackDrive.setPower(-.7);
        //robot.LeftFrontDrive.setPower(.9);
        //robot.LeftBackDrive.setPower(.9);
        //robot.RightFrontDrive.setPower(.9);
        //robot.RightBackDrive.setPower(.9);
        //sleep(1000);
        //robot.LeftFrontDrive.setPower(.9);
        //robot.LeftBackDrive.setPower(.9);
        //robot.RightFrontDrive.setPower(.9);
        //robot.RightBackDrive.setPower(.9);
        //sleep(1000);
        //robot.LeftFrontDrive.setPower(.9);
        //robot.LeftBackDrive.setPower(.9);
        //robot.RightFrontDrive.setPower(.9);
        //robot.RightBackDrive.setPower(.9);
        //sleep(1000);

        //multiply 133 for how many inchs robot to travel













    }
}