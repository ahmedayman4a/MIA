#include <sys/types.h>
#include <ros.h>
class Encoder
{
private:
    double resolution;
    long long previous_count;
    
public:
    Encoder(double resolution);
    double count_to_rpm(u_int32_t current_count,double dt, ros::NodeHandle& nh);
};
