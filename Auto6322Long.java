package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class Auto6322Long extends RoverRuckusLinearOpMode6322{

    public void runOpMode() {
        initialize();
        
        waitForStart();
        //come down
        /*  lift.setPower(-1);
        sleep(1500);
            lift.setPower(0);*/
        //wait for friends to pass
        
        //go to depot
        Strafe(18,.3);
        sleep(100);
        Forward(125,.5);
        turnWithoutGyro(-25,1); //113.5 is 180
        Forward(30,1);
        Strafe(-55,1);
        turnWithoutGyro(45,1);
        intake.setPower(-1);
        sleep(1000);
        intake.setPower(0);
        //add parking?

    }
}