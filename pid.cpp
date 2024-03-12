
class Pid {
public:
  double kp;
  double ki;
  double kd;

  unsigned long lastTime;
  float lastErrorCalc;
  double cumulativeError;

  Pid(double kpIn, double kiIn, double kdIn) {
    lastTime = millis();
    lastErrorCalc = 0;
    cumulativeError = 0;

    kp=kpIn;
    ki=kiIn;
    kd=kdIn;
  }

  void resetPid() {
    lastTime = millis();
    lastErrorCalc = 0;
    cumulativeError = 0;

  }

  int getAdjustment(float setpoint, float input) {
    unsigned long currentTime = millis();
    double deltaTime = (double)(currentTime - lastTime);
    lastTime = currentTime;

    float error = input - setpoint;

    if (error < minError) return 0;

    float prop = kp * error;
    
    cumulativeError += error * deltaTime;
    float integral = cumulativeError * ki;

    float derivative = ((error - lastErrorCalc) / deltaTime) * kd;
    lastErrorCalc = error;

    int adjustment = prop + integral + derivative;


    Serial.println("e: " + String(error) + " p: " + String(prop) + " i: " + String(integral) +  " d: " + String(derivative) + " sum: " +  String(adjustment));
    return adjustment;

  }
  