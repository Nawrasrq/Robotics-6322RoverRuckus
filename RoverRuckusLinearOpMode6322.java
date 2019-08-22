package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Map;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="6322RoverRuckus", group="LinearOpMode")

public abstract class RoverRuckusLinearOpMode6322 extends LinearOpMode{

    //public objects
    public double globalAngle = .30;
    public double P = 1;
    public double I = 1;
    public double D = 1;

    public double integral, derivative, previous_error, error, rcw = 0;
    public BNO055IMU Imu;
    public Orientation lastAngles = new Orientation();

    //Mechanum 4 wheel drivetrain
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    //arm
    public DcMotor intake;
    public DcMotor arm1;
    public DcMotor lift;
    public DcMotor linearSlide;

    //public Servo push1;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     DRIVE_GEAR_REDUCTION    = 1;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI) * .66;

    public void initialize(){
        //hardwareMapping

        //drivetrain
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");

        //reverse
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //intake
        intake = hardwareMap.dcMotor.get("intake");
        arm1 = hardwareMap.dcMotor.get("arm1");
        lift = hardwareMap.dcMotor.get("lift");
        linearSlide = hardwareMap.dcMotor.get("ls");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        //PID/IMU

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        Imu = hardwareMap.get(BNO055IMU.class, "imu");
        Imu.initialize(parameters);
    }

    //Working Forward/Backward method using encoders and motorPower
    public void Forward(double inches, double power){
        if(opModeIsActive())  {
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setTargetPosition((int)(inches * COUNTS_PER_INCH));
            backRight.setTargetPosition((int)(inches * COUNTS_PER_INCH));
            frontLeft.setTargetPosition((int)((inches * COUNTS_PER_INCH)));
            frontRight.setTargetPosition((int)(inches * COUNTS_PER_INCH));

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && backRight.isBusy() && frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()){
                telemetry.addData("Path1",  "Running to %7d :%7d", backLeft.getTargetPosition(), backRight.getTargetPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.update();

                backLeft.setPower(power);
                backRight.setPower(power);
                frontLeft.setPower(power);
                frontRight.setPower(power);
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    //Working Strafe method using encoders and motorPower
    //multiply target position inches by 2 to not lose power when strafing
    public void Strafe(double inches, double power){
        if(opModeIsActive())  {
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setTargetPosition((int)(inches * COUNTS_PER_INCH));
            backRight.setTargetPosition((int)(-inches * COUNTS_PER_INCH));
            frontLeft.setTargetPosition((int)((-inches * COUNTS_PER_INCH)));
            frontRight.setTargetPosition((int)(inches * COUNTS_PER_INCH));

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && backRight.isBusy() && frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()){
                telemetry.addData("Path1",  "Running to %7d :%7d", backLeft.getTargetPosition(), backRight.getTargetPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.update();

                backLeft.setPower(power);
                backRight.setPower(power);
                frontLeft.setPower(power);
                frontRight.setPower(power);
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    //Turn method not using GyroSensor.
    public void turnWithoutGyro(double inches, double power){
        if(opModeIsActive())  {
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setTargetPosition((int)(inches * COUNTS_PER_INCH));
            backRight.setTargetPosition((int)(-inches * COUNTS_PER_INCH));
            frontLeft.setTargetPosition((int)(inches * COUNTS_PER_INCH));
            frontRight.setTargetPosition((int)(-inches * COUNTS_PER_INCH));

            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && backRight.isBusy() && frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()){

                telemetry.addData("Path1",  "Running to %7d :%7d", backLeft.getTargetPosition(), backRight.getTargetPosition());
                telemetry.addData("Path2",  "Running at %7d :%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition());
                telemetry.update();

                backLeft.setPower(power);
                backRight.setPower(power);
                frontLeft.setPower(power);
                frontRight.setPower(power);
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void MoveArm(double degrees, double power){
        if(opModeIsActive())  {
            arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm1.setTargetPosition((int)(degrees));

            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while(opModeIsActive() && arm1.isBusy()){
                telemetry.addData("Path1", arm1.getTargetPosition());
                telemetry.addData("Path2", arm1.getCurrentPosition());
                telemetry.update();

                arm1.setPower(power);
            }

            arm1.setPower(0);

            arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void stopDrive(){
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    private double checkDirection(){
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;
        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;
        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void ForwardByGyro(double distance, double initAngle, double power){
        double targetAngle = initAngle;
        double newEncTarget = 0;

        if(opModeIsActive()){
            newEncTarget = distance - ((backLeft.getCurrentPosition() + backRight.getCurrentPosition() + frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()) / 4);

            if(getAngle() < 0.5|| getAngle() > - 0.5){
                Forward(distance, power);
            }
            else{
                rotate(-getAngle(), 0.15);
                sleep(100);
                Forward(newEncTarget, power);
            }
        }
    }
    private void resetAngle(){
        lastAngles = Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
    public void getPID(double sensorValue, double desiredValue){
        resetAngle();

        error = (desiredValue - sensorValue);
        this.integral += (error * 0.2);
        derivative = (error - this.previous_error) / 0.2;
        this.rcw = P * error + I * this.integral + D * derivative;
    }
}
