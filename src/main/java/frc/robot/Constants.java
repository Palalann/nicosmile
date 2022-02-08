// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DIFFDRIVE{
       public static final int RMMOTOR = 1;
       public static final int RFMOTOR = 2;
       public static final int LMMOTOR = 3;
       public static final int LFMOTOR = 4;
    }
    public static final class SIMDB{
        public static final int RMMOTOR_ID = 1;
        public static final int RFMOTOR_ID = 2;
        public static final int LMMOTOR_ID = 3;
        public static final int LFMOTOR_ID = 4; 
    }
    public static final class OMNIX{
        public static final int LFMOTOR = 1;
        public static final int RFMOTOR = 2;
        public static final int LBMOTOR = 3;
        public static final int RBMOTOR = 4;
    }
    public static final class OMNIH{
        public static final int VERTMASTER_MOTOR1_ID = 1;
        public static final int VERTMASTER_MOTOR2_ID = 2;
        public static final int HORIMASTER_MOTOR1_ID = 3;
        
    }
    public static class MEASUREMENT{
        public static final int TRACKWIDTH = 2; //meters
        public static final int WHEEL_RADIUS = 2; //meters
        public static final int ENCODER_RESOLUTION = 2; //meters

        public static final int kMaxAngularSpeed = 2;
        public static final int kMaxSpeed = 2;
    }
    public static final class PiAiDi{
        public static final int kP = 1;
        public static final int kI = 1;
        public static final int kD = 1;
        
        public static final int kPL = 1;
        public static final int kIL = 1;
        public static final int kDL = 1;

        public static final int kPR = 1;
        public static final int kIR = 1;
        public static final int kDR = 1;

        public static final int kToleranceDegree = 1;
        public static final int kAngularVelocity = 1;
    }
    public static final class Joystick{
        public static final int Sq = 1;
        public static final int X = 2;
        public static final int O = 3;
        public static final int Tri = 4;
       
        public static final int L1= 5;
        public static final int R1 = 6;
        public static final int L2 = 7;
        public static final int R2 = 8;

        public static final int L3 = 11;
        public static final int R3 = 12;

        public static final int Share = 9;
        public static final int Option = 10;
        public static final int TouchPad = 13;
         
        public static final int Up = 14;
        public static final int Down = 15;
        public static final int Left = 16;
        public static final int Right = 17;
               
    }
    public static final class DriveConstants{
        public static final double ks = 1; //Volt
        public static final double kv= 1; //Volt x second / meter
        public static final double ka = 1; //Volt x second^2 / meter
        
        public static final double kP = 1; //Volt x second / meter, drive velocity
        
    }
    public static final class AutoConstants{
        public static final int kMaxSpeedMetersPerSecond = 1;
        public static final int kMaxAccelerationMetersPerSecondSquared = 1;
        
    }
    public static final class RAMSETE{
        public static final int kRamseteB = 1;
        public static final int kRamseteZeta = 1;
    }
    public static final class TRAJECTORY{
        public static final double maxVelo = 1;
        public static final double maxAcce = 2; 
        public static final double startVelo = 1;
        public static final double endVelo1 = 2;
        public static final double endVelo2 = 4;
    }
    public static final class PATH2{
        public static final double start2_x = 2.0;
        public static final double start2_y = -5.2296;
        public static final double start2_thetha = 0;

        public static final double end2_x = 5.259754110694355;
        public static final double end2_y = -6.233897531790321;
        public static final double end2_thetha = -0.5318410055804588;

        public static final double point21_x = 2.642536530601042;
        public static final double point21_y = -5.506115103101271;
        

        public static final double point22_x = 3.244356615863141;
        public static final double point22_y = -5.758039789955173;

        public static final double point23_x = 4.210067915469764;
        public static final double point23_y = -5.268186232183697;
        
        public static final double point24_x = 4.853875448540847;
        public static final double point24_y = -5.282182048120024;

        public static final double point25_x = 5.343729006312322;
        public static final double point25_y = -5.66006907840088;

        public static final boolean isReverse2 = true;
        
    }
    public static final class PATH1{
        public static final double start1_x = 6.0;
        public static final double start1_y = -3.2295999999999996;
        public static final double start1_thetha = 0;

        public static final double end1_x = 2.0;
        public static final double end1_y = -5.2296;
        public static final double end1_thetha = -0.0;

        public static final double point11_x = 4.503980050132651;
        public static final double point11_y = -3.1688138417345164;

        public static final double point12_x = 3.090402640563534;
        public static final double point12_y = -3.5327050560790414;

        public static final double point13_x = 1.5228712556948119;
        public static final double point13_y = -4.050550245723172;

        public static final double point14_x = 0.9770344341780249;
        public static final double point14_y = -4.820320122221204;

        public static final boolean isReverse1 = true;
        
    }
    public static final class PATH3{
        public static final double start3_x = 1;
        public static final double start3_y = 1;
        public static final double start3_thetha = 1;

        public static final double end3_x = 1;
        public static final double end3_y = 1;
        public static final double end3_thetha = 1;

        public static final double point31_x = 1;
        public static final double point31_y = 1;

        public static final double point32_x = 1;
        public static final double point32_y = 1;

        public static final boolean isReverse3 = true;
        
    }
}
