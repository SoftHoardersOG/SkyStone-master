package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

@Autonomous(name = "Parcare", group = "Linear OpMode")

public class Parcare extends LinearOpMode {

    Hardware robot = new Hardware();

    final double TICKSURI_PER_CM = 27.3696374577;
    private double inc1;
    private double inc2;

    public enum Direction{
        FRONT, BACK, LEFT, RIGHT;
    }

    public void mersTicksuri(double distance, Direction dir, double pow, DcMotor motor, double ticks,
                             double power, boolean brate, boolean colectare, boolean tata){
        double curent=0, minpow=0,power1=0,power2=0;

        if(motor!=null)
        {
            curent = motor.getCurrentPosition();
            minpow = 0;

            power1 = power + 0.2;
            power2 = power - 0.3;
            if (motor == robot.rotatie) {
                if (ticks <= 350 && ticks >= 40)
                    minpow = 0.2;
                if (ticks >= 710 && ticks <= 1300)
                    minpow = -0.2;
            }

            if (motor == robot.glisisus)
                minpow = -0.15;
        }
        double fl,fr,br,bl;
        double speed = -1 * pow;
        fl = fr = br = bl = -1;
        if(dir == Direction.BACK)
            fl = fr = br = bl = 1;
        else if(dir == Direction.LEFT)
            fl = br = 1;
        else if(dir == Direction.RIGHT)
            fr = bl = 1;
        double initial = robot.back_left.getCurrentPosition();
        double necesar = distance * TICKSURI_PER_CM;
        double rem = 15 * TICKSURI_PER_CM;    /// decelerez pe ultimii 15 cm
        robot.front_left.setPower(speed * fl);
        robot.front_right.setPower(speed * fr);
        robot.back_right.setPower(speed * br);
        robot.back_left.setPower(speed * bl);
        boolean ver  = true;
        boolean b1 = false, b2= false;
        boolean mot = true;

        while((Math.abs(robot.back_left.getCurrentPosition() - initial)<necesar && (Math.abs(robot.m.getVelocity()) > 60 || ver))
                || (brate && (!b1 || !b2)) ||(motor!=null && mot)
                && !isStopRequested() && opModeIsActive() ){
            telemetry.addData("in MersTicks ", 1);
            telemetry.addData("b1", b1);
            telemetry.addData("B2", b2);
            telemetry.addData("rotatie", robot.rotatie.getCurrentPosition());
            telemetry.update();
            if(colectare)
            {
                if(robot.a1.getVelocity() > -100)
                    inc1 = -0.7;
                else
                    inc1 = 0;

                if(robot.a2.getVelocity() < 100)
                    inc2 = 0.7;
                else
                    inc2 = 0;

                robot.aspira1.setPower(-0.3 + inc1);
                robot.aspira2.setPower(0.3 + inc2);
                robot.bagas.setPower(-1);
                robot.bagad.setPower(1);

                telemetry.addData("a1 velocity", robot.a1.getVelocity());
                telemetry.addData("a2 velocity", robot.a2.getVelocity());
                telemetry.addData("a1 power", robot.aspira1.getPower());
                telemetry.addData("a2 power", robot.aspira2.getPower());
                telemetry.update();
            }
            if(brate){
                if (robot.atins1.getState()){
                    robot.cs.setPower(1);
                }
                else{
                    b1 = true;
                    robot.cs.setPower(0);
                }
                if (robot.atins2.getState())
                    robot.cd.setPower(-1);
                else{
                    b2 = true;
                    robot.cd.setPower(0);
                }
            }
            if(motor!=null) {

                if ((curent <=ticks && motor.getCurrentPosition() >= ticks)||(curent >=ticks && motor.getCurrentPosition() <= ticks)) {
                    motor.setPower(minpow);
                    mot = false;
                }
                else {
                    if (curent <= ticks) {
                        if (motor == robot.rotatie) {
                            if (robot.rotatie.getCurrentPosition() >= 550) {
                                power = power2;
                            } else {
                                power = power1;
                            }
                        }
                        motor.setPower(power);

                    } else {
                        if (motor == robot.rotatie && robot.rotatie.getCurrentPosition() >= 550) {
                            power = power1;
                        }
                        motor.setPower(-power);
                    }
                }
            }
            double d = necesar - Math.abs(robot.back_left.getCurrentPosition() - initial);
            if(d < rem){
                ver=false;
                double v = -1 * pow * d / rem;
                if(v>-0.47 * pow)
                    v=0;
                robot.front_left.setPower(v * fl);
                robot.front_right.setPower(v * fr);
                robot.back_left.setPower(v * bl);
                robot.back_right.setPower(v * br);
            }
        }


        if(colectare && !isStopRequested() && opModeIsActive()) {
            while(robot.distanta.getDistance(MM)>60 && opModeIsActive() && !isStopRequested()){
                if(robot.a1.getVelocity() > -100)
                    inc1 = -0.7;
                else
                    inc1 = 0;

                if(robot.a2.getVelocity() < 100)
                    inc2 = 0.7;
                else
                    inc2 = 0;

                if(robot.distanta.getDistance(MM) <270){
                    inc1 = 1;
                    inc2 = -1;
                }
                robot.aspira1.setPower(-0.3 + inc1);
                robot.aspira2.setPower(0.3 + inc2);

                robot.bagas.setPower(-1);
                robot.bagad.setPower(1);
            }

            robot.aspira1.setPower(0);
            robot.aspira2.setPower(0);
            robot.bagas.setPower(0);
            robot.bagad.setPower(0);
        }

        if(tata && !isStopRequested() && opModeIsActive()) {
            if(robot.brat.getPosition() == 0.9)
                robot.brat.setPosition(0.2);
            else
                robot.brat.setPosition(0.9);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addLine("Robot is initialised");
        telemetry.update();

        waitForStart();

        mersTicksuri(20, Direction.FRONT, 1, null,0,0,true, false,false);

    }
}
