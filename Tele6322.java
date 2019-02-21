package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.File;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.ftccommon.SoundPlayer;


@TeleOp(name="Tele6322", group="LinearOpMode")

public class Tele6322 extends RoverRuckusLinearOpMode6322{

    //objects
    double dpL = 0;
    double dpR = 0;
    double dpD = 0;
    double dpU = 0;

    int a = 0;
    int b = 0;
    int c = 0;
    int d = 0;

    @Override
    public void runOpMode(){

        initialize();

        while(!(isStarted() || isStopRequested())){

            idle();

        }
        waitForStart();
        while(opModeIsActive()){

            float Ly = -gamepad1.left_stick_y;
            float Lx = gamepad1.left_stick_x;
            float Rx = gamepad1.right_stick_x;

            /*frontLeft.setPower((Ly-Lx) + (-Rx * .5));
            frontRight.setPower(Ly+Lx + (Rx * .5));
            backLeft.setPower(Ly+Lx + (-Rx * .5));
            backRight.setPower(Ly-Lx + (Rx * .5));*/
            
            frontLeft.setPower((Rx*.5) + (Ly+Lx));
            frontRight.setPower((-Rx*.5) + (Ly-Lx));
            backLeft.setPower((Rx*.5)+ (Ly-Lx));
            backRight.setPower((-Rx*.5)+(Ly+Lx));

            telemetry.addData("Arm Enc Counts: ", arm1.getCurrentPosition());
            telemetry.update();

            //lift
            if(gamepad1.dpad_up){
                lift.setPower(-1);
            }
            else if(gamepad1.dpad_down){
                lift.setPower(1);
            }
            else{
                lift.setPower(0);
            }

            //intake
            
            if(gamepad1.a && a == 0){
                a = 1;
            }
            else if(!gamepad1.a && a == 1){
                intake.setPower(1);
                a = 2;
            }
            else if(gamepad1.a && a == 2){
                a = 3;
            }
            else if(!gamepad1.a && a == 3){
                intake.setPower(0);
                a = 0;
            }
            if(gamepad1.b && b == 0){
                b = 1;
            }
            else if(!gamepad1.b && b == 1){
                intake.setPower(-1);
                b = 2;

            }
            else if(gamepad1.b && b == 2){

                b = 3;

            }
            else if(!gamepad1.b && b == 3){
                intake.setPower(0);
                b = 0;
            }

            //Extendo arm
            if(gamepad1.left_trigger > 0.05){
                linearSlide.setPower(gamepad1.left_trigger);
            }
            else if(gamepad1.right_trigger > 0.05){
                linearSlide.setPower(-gamepad1.right_trigger);
            }
            else{
                linearSlide.setPower(0);
            }
//Arm threshold would work like- if(!gamepad1.left_trigger && lastEncoderticks < .getEncoderticks) move back, flip the equality sign
                                                                                                // to move forward to counter gravity 

            //move arm forwards and backwards
            if(gamepad1.y){
                arm1.setPower(-.8);
            }
            else if(gamepad1.x){
                arm1.setPower(.8);
            }
            else {
                arm1.setPower(-0.12);
            }

            /*else{

                push1.setPosition(1);
                push2.setPosition(0.1);

            }*/
        }



        // this is to make the dpad mimick a float value as best as i could (on a whim)

        //DPad config that isn't in use yet -- the correlation to the motors is commented below

/*          boolean DL = gamepad1.dpad_left; //DL represents the left button on the dpad (respective for the rest)
            boolean DR = gamepad1.dpad_right;
            boolean DD = gamepad1.dpad_down;
            boolean DU = gamepad1.dpad_up;

            dpadGrowth(DL, dpL);
            dpadGrowth(DR, dpR);
            dpadGrowth(DD, dpD);
            dpadGrowth(DU, dpU);
*/

        //DPad config for drivetrain
        /*  frontLeft.setPower((dpU - dpD)+(dpR - dpL));
            frontRight.setPower((dpU - dpD)-(dpR - dpL));
            backLeft.setPower((dpU - dpD)+(dpR - dpL));
            backRight.setPower((dpU - dpD)-(dpR - dpL));
            */

/*    public void dpadGrowth(dpX, dpY){
            while(dpX){

                dpY += 0.33333;
            }
            if(!dpX){
          I
                dpY = 0;
            }
    }*/

    }
}
