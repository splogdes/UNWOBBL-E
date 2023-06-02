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
            Serial.print("Coefficients Ki: ");Serial.print(ki);Serial.print(", Kd: ");Serial.print(kd);Serial.print(", Kp: ");Serial.print(kp);Serial.print(", Ts: ");Serial.println(ts);
        }
        void set_coefficients(double Kp,double Ki,double Kd,double Ts){
            ki = Ki;
            kp = Kp;
            kd = Kd;
            ts = Ts;
        }
};

class Kalman_filter_pitch {
    private:
        double theta[2];
        double P[2][2];
        double y;
        double K[2];
        double Q[2];
        double R;
        double S;
    public:
        Kalman_filter_pitch(double Q_theta,double Q_theta_rate,double R_variance) {
            theta[0] = 0;
            theta[1] = 0; 
            P[0][0] = 1000;
            P[0][1] = 0;
            P[1][0] = 0;
            P[1][1] = 1000;
            K[0] = 0;
            K[1] = 0;
            R = R_variance;
            Q[0] = Q_theta;
            Q[1] = Q_theta_rate;
        }
        double filter(double angle_rate,double pitch,double delta_t){
            theta[0] += delta_t*(angle_rate-theta[1]);
            P[0][0] += delta_t*(delta_t*P[1][1]-P[0][1]-P[1][0]+Q[0]);
            P[0][1] -= delta_t*P[1][1];
            P[1][0] -= delta_t*P[1][1];
            P[1][1] += delta_t*Q[1];
            y = pitch - theta[0];
            S = P[0][0] + R;
            K[0] = P[0][0]/S;
            K[1] = P[1][0]/S;
            theta[0] += K[0]*y; 
            P[1][0] -= K[1]*P[0][0];
            P[1][1] -= K[1]*P[0][1];
            P[0][0] -= K[0]*P[0][0];
            P[0][1] -= K[0]*P[0][1];
            return theta[0];
        }
};