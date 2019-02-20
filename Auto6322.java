package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class Auto6322 extends RoverRuckusLinearOpMode6322 {
    //crater side testing
    public void runOpMode() {
                initialize();
                waitForStart();
                telemetry.addLine("going center");
                
                Strafe(13,.5);
                sleep(100);
                Strafe(-5,.5);
                sleep(100);
                Forward(36,.5);
                sleep(100);
                rotate(20,.5); // check rotation amount, point to depot
                sleep(1000);
                
                //to depot
                Forward(60,.5); //check forward direction
                intake.setPower(-1);
                sleep(1000);
                intake.setPower(0);
                Forward(-70,.5);
                
        //36in 60in


    }
}