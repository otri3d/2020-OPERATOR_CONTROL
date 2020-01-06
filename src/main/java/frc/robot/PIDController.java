package frc.robot;

public class PIDController{
    
    double pGain;
    double iGain;
    double dGain;

    double pOut;
    double iOut;
    double dOut;

    double error;
    double errorSum = 0;
    double lastError = 0;

    double dProcessVar;
     
    double output;
    double previousValue = 0;
    double previousAverage = 0;
    double currentAverage;
    double average;

    boolean atTarget;

    public PIDController (double p, double i, double d){
        this.errorSum = 0;
        this.lastError = 0;
        this.pGain = p;
        this.iGain = i;
        this.dGain = d; 
    }

    public void resetIntegral(){
        this.errorSum = 0.0;
    }

    public void resetDerivative(){
        this.lastError = 0.0;
    }

    public void resetPID(){
        resetDerivative();
        resetIntegral();
        this.atTarget = false;
    }

    public void changePIDGains(double kP, double kI, double kD){
        this.pGain = kP;
        this.iGain = kI;
        this.dGain = kD;
    }

    public double calcPID(double setPoint, double currentValue, double epsilon){
        this.error = setPoint - currentValue;

        if (Math.abs(error)<= epsilon){
            atTarget = true;
        }
        else{
            atTarget = false;
        }

        this.pOut = this.pGain * this.error;

        this.errorSum += error;
        this.iOut = this.iGain * this.errorSum;

        this.dProcessVar = this.error - this.lastError;
        this.dOut = this.dGain * this.dProcessVar;

        lastError = error;

        this.output = this.pOut + this.iOut + this.dOut;

        if (output != 0.0){
            this.output = output/Math.abs(output) * (1.0-Math.pow(0.1, Math.abs(output)));
        }
        return output;
    }
}