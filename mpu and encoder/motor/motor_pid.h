class MotorPID
{
private:
    double clamp_saturation(double controlled_rpm);
    float Kp;
    float Ki;
    float Kd;
    int target_rpm;
    int rpm_clamp_limit;
    float eIntegral;
    float last_rpm;

public:
    MotorPID(int target_rpm, float Kp, float Ki, float Kd);

    MotorPID(int target_rpm, float Kp, float Ki, float Kd, int rpm_clamp_limit);

    double get_controlled_rpm(double rpm, double dt);
};
