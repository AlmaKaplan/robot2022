
package frc.robot;

public class PortMap {

    public class IntakeRollers {
        public static final int INTAKE_MOTOR = 6;

        public static final int INTAKE_ARM_DIGITAL_INPUT_SENSOR = 3;
    }

    public class IntakeArm {
        public static final int INTAKE_ARM_MOTOR = 5;
    }

    public class Transfer {
        public static final int TRANSFER_MOTOR = 8;

        public static final int TRANSFER_FIRST_DIGITAL_INPUT_SENSOR = 1;
        public static final int TRANSFER_SECOND_DIGITAL_INPUT_SENSOR = 2;

    }

    public class Shooter {
        public static final int SHOOTER_LEFT_MOTOR = 10;
        public static final int SHOOTER_RIGHT_MOTOR = 12;
    }

    public class DriveTrain {
        public static final int DRIVE_TRAIN_LEFT_MOTOR = 3;
        public static final int DRIVE_TRAIN_RIGHT_MOTOR = 2;
    }
}
