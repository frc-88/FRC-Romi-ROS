package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Timer;


public class RomiNetworkTables {
    NetworkTable accel_table;
    NetworkTableEntry imu_time;
    NetworkTableEntry accel_x;
    NetworkTableEntry accel_y;
    NetworkTableEntry accel_z;

    NetworkTable gyro_table;
    NetworkTableEntry gyro_x;
    NetworkTableEntry gyro_y;
    NetworkTableEntry gyro_z;

    NetworkTable encoder_table;
    NetworkTableEntry encoder_left;
    NetworkTableEntry encoder_right;

    NetworkTable motor_table;
    NetworkTableEntry motor_time;
    NetworkTableEntry motor_left;
    NetworkTableEntry motor_right;

    public RomiNetworkTables()
    {
        NetworkTableInstance inst = NetworkTableInstance.create();
        inst.startClient("192.168.0.15");
        inst.setUpdateRate(0.01);
        NetworkTable table = inst.getTable("ROS");


        accel_table = table.getSubTable("accel");
        imu_time = accel_table.getEntry("time");
        accel_x = accel_table.getEntry("x");
        accel_y = accel_table.getEntry("y");
        accel_z = accel_table.getEntry("z");

        gyro_table = table.getSubTable("gyro");
        gyro_x = gyro_table.getEntry("x");
        gyro_y = gyro_table.getEntry("y");
        gyro_z = gyro_table.getEntry("z");

        encoder_table = table.getSubTable("encoder");
        encoder_left = encoder_table.getEntry("left");
        encoder_right = encoder_table.getEntry("right");

        motor_table = table.getSubTable("motors");
        motor_time = motor_table.getEntry("time");
        motor_left = motor_table.getEntry("left");
        motor_right = motor_table.getEntry("right");
    }

    public void setImuTime(double time)
    {
        imu_time.setDouble(time);
    }

    public void setAccel(double x, double y, double z)
    {
        accel_x.setDouble(x);
        accel_y.setDouble(y);
        accel_z.setDouble(z);
    }

    public void setGyro(double x, double y, double z)
    {
        gyro_x.setDouble(x);
        gyro_y.setDouble(y);
        gyro_z.setDouble(z);
    }

    public void setEncoders(double left, double right)
    {
        encoder_left.setDouble(left);
        encoder_right.setDouble(right);
    }

    public double getLeftMotor()
    {
        return motor_left.getDouble(0.0);
    }

    public double getRightMotor()
    {
        return motor_right.getDouble(0.0);
    }

    public boolean didMotorCmdUpdated()
    {
        double dt = Timer.getFPGATimestamp() - motor_time.getDouble(0.0);
        return dt < 0.5;
    }
}
