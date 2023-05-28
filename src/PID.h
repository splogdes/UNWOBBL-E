#include <Arduino.h>


class PID {
    private:
        double ki;
        double kp;
        double kd;
        double ts;
        double y;
        double x[2];
        double sat;
    public:
        PID(double Kp,double Ki,double Kd,double Ts){
            ki = Ki;
            kp = Kp;
            kd = Kd;
            ts = Ts;
            sat = infinity();
        }
        PID(double Kp,double Ki,double Kd,double Ts,double Saturation){
            ki = Ki;
            kp = Kp;
            kd = Kd;
            ts = Ts;
            sat = Saturation;
        }
        // Initiate values to 0
        void init(){
            x[0] = 0;
            x[1] = 0;
            y = 0;
        }
        // Get PID output
        double filter(double u){
            double tmp;
            tmp = (kd/ts)*x[1] - (kp + 2*(kd/ts))*x[0] + (kp + ki*ts + kd/ts)*u + y;
            if(sat < fabs(tmp)){
                if(tmp>0){
                    y = sat;
                }else{
                    y = -sat;
                }
            }else{
                y = tmp;
            }
            x[1] = x[0];
            x[0] = u;
            return y;
        }
        // Prints Coefficients
        void print_coefficients(){
            Serial.print("Coefficients Ki: ");
            Serial.print(ki);
            Serial.print(", Kd: ");
            Serial.print(kd);
            Serial.print(", Kp: ");
            Serial.print(kp);
            Serial.print(", Ts: ");
            Serial.println(ts);
        }
        void set_coefficients(double Kp,double Ki,double Kd,double Ts){
            ki = Ki;
            kp = Kp;
            kd = Kd;
            ts = Ts;
        }
};