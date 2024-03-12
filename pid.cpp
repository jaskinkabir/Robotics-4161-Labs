class Pid {
public:
  double kp;
  double ki;
  double kd;

  unsigned long lastTime;
  int lastError;
  double cumulativeError;

  Pid(double kp, double ki, double kd) {
    lastTime = millis();
    lastError = 0;
    cumulativeError = 0;
  }

  int getAdjustment(int setpoint, int input) {
    unsigned long currentTime = millis();
    double deltaTime = (double)(currentTime - lastTime);
    lastTime = currentTime;

    int error = input - setpoint;

    float prop = kp * error;
    cumulativeError += error * deltaTime;

    float integral = cumulativeError * ki;

    float derivative = ((error - lastError) / deltaTime) * kd;

    lastError = error;

    int adjustment = prop + integral + derivative;


    //Serial.println("e: " + String(error) + " p: " + String(prop) + " i: " + String(integral) +  " d: " + String(derivative) + " sum: " +  String(adjustment));
    return adjustment;

  }
  
};