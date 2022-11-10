#include <sys/_stdint.h>
class TimeHelper{
    private:
    uint32_t previous_time;
    public:
    TimeHelper();

    /// @brief uses millis() to get delta time in seconds since the previous get_dt() call
    /// @return the delta time since the last get_dt() call
    double get_dt();
};
