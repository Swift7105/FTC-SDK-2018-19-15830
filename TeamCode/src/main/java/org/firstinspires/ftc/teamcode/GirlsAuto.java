
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Pushbot: GirlsAuto", group = "Pushbot")
//@Disableds
public class GirlsAuto extends LinearOpMode {

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
        sleep(800);
        robot.LeftFrontDrive.setPower(.2);
        robot.LeftBackDrive.setPower(.2);
        robot.RightFrontDrive.setPower(.2);
        robot.RightBackDrive.setPower(.2);
        sleep(3500);
        robot.LeftFrontDrive.setPower(.3);
        robot.LeftBackDrive.setPower(.3);
        robot.RightFrontDrive.setPower(0);
        robot.RightBackDrive.setPower(0);
        sleep(2000);
        robot.LeftFrontDrive.setPower(.3);
        robot.LeftBackDrive.setPower(.3);
        robot.RightFrontDrive.setPower(.3);
        robot.RightBackDrive.setPower(.3);
        sleep(4000);
        robot.LeftFrontDrive.setPower(.1);
        robot.LeftBackDrive.setPower(.1);
        robot.RightFrontDrive.setPower(-.1);
        robot.RightBackDrive.setPower(-.1);
        sleep(200);
        robot.Arm.setPower(-1);
        sleep(1000);
        robot.Arm.setPower(.1);
        sleep(1000);
        robot.LeftFrontDrive.setPower(-.3);
        robot.LeftBackDrive.setPower(-.3);
        robot.RightFrontDrive.setPower(-.5);
        robot.RightBackDrive.setPower(-.5);
        sleep(500);
        robot.LeftFrontDrive.setPower(-.8);
        robot.LeftBackDrive.setPower(-.8);
        robot.RightFrontDrive.setPower(-.9);
        robot.RightBackDrive.setPower(-.9);
        sleep(3000);





        //multiply 133 for how many inches robot to travel













    }
}