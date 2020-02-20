package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="Teste_Autonomie",group = "ma_dsitrez")

@Disabled
public class Teste_autonomie extends LinearOpMode {

    Hardware robot = new Hardware();

    double TICKSURI_PER_CM = 28;

    public enum Direction{
        FRONT,BACK,LEFT,RIGHT;
    }

    public void mersSimultan(Direction dir,double distance, double powMers, DcMotor [] motor,double [] putere,int [] tickuri,
                             Servo ser, int ordine,double poz)
    {
        double fl,fr,br,bl;
        fl = fr = br = bl = powMers;
        if(dir == Direction.BACK)
            fl = fr = br = bl = -powMers;
        else if(dir == Direction.LEFT)
            fl = br = -powMers;
        else if(dir == Direction.RIGHT)
            fr = bl = -powMers;
        double initial = robot.back_left.getCurrentPosition();
        double necesar = distance * TICKSURI_PER_CM;
        robot.front_left.setPower(fl);
        robot.front_right.setPower(fr);
        robot.back_right.setPower(br);
        robot.back_left.setPower(bl);

        if(motor[1].getCurrentPosition()<tickuri[1])
          motor[1].setPower(putere[1]);
        else
            motor[1].setPower(-putere[1]);


        int status = 1;
        

        while((Math.abs(robot.back_left.getCurrentPosition() - initial)<necesar || status< putere.length) && !isStopRequested() && opModeIsActive()){

            if(robot.back_left.getCurrentPosition() - initial>necesar)
                     robot.stopRobot();

            if(putere[status]>0 && motor[status].getCurrentPosition() >= tickuri[status])
            {
                motor[status].setPower(minpow(motor[status],tickuri[status]));

                status++;

                if(motor[status].getCurrentPosition()<tickuri[status])
                    motor[status].setPower(putere[status]);
                else
                    motor[status].setPower(-putere[status]);

            }
            else if(putere[status]<0 && motor[status].getCurrentPosition() <= tickuri[status])
            {
                motor[status].setPower(0);

                 status++;

                if(motor[status].getCurrentPosition()<tickuri[status])
                    motor[status].setPower(putere[status]);
                else
                    motor[status].setPower(-putere[status]);
            }

            if(status==ordine+1 && ser!=null)
                ser.setPosition(poz);

            telemetry.addData("Status ",status);
            telemetry.addData("Number of actions ",putere.length-1);
            telemetry.update();
        }

    }

    public double minpow(DcMotor motori, int ticks)
    {
        double min=0;
        if (motori == robot.rotatie) {
            if (ticks <= 350 && ticks >= 40)
                min = 0.2;
            if (ticks >= 710 && ticks <= 1300)
                min = -0.2;
        }

        if (motori == robot.glisisus)
            min = -0.15;

        return min;
    }




    @Override
    public void runOpMode() throws InterruptedException
    {

        robot.init(hardwareMap);

        waitForStart();

        DcMotor [] motoare1 = {null,robot.glisisus,robot.rotatie,robot.glisisus,robot.glisisus,robot.rotatie,robot.glisisus,robot.rotatie};
        double [] puteri1 = {0,1,0.5,0.8,1,0.8,0.8,0.7};
        int [] ticksuri1 = {0,-900,1500,-800,-1000,300,0,0};

        mersSimultan(Direction.BACK,60,1,motoare1,puteri1,ticksuri1,robot.brat,3,0.2);

        robot.stopRobot();
    }

}