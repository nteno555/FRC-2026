package frc2025.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {

    @AutoLog
    class LedIOInputs {
        public boolean candleConnected = false;
        public double supplyVoltage = 0.0;
        public double outputCurrent = 0.0;
        public double deviceTempCelsius = 0.0;
    }

    public void updateInputs(LedIOInputs inputs);
    public void setSolidColor(int startIndex, int endIndex, int r, int g, int b);
    public void setRainbow(int startIndex, int endIndex, double brightness, double frameRate, int slot);
    public void setLarson(int startIndex, int endIndex, int r, int g, int b, int size, double frameRate, int slot);
    public void setStrobe(int startIndex, int endIndex, int r, int g, int b, double frameRate, int slot);
    public void setSingleFade(int startIndex, int endIndex, int r, int g, int b, double frameRate, int slot);
    public void setRgbFade(int startIndex, int endIndex, double brightness, double frameRate, int slot);
    public void setFire(int startIndex, int endIndex, double brightness, double sparking, double cooling, double frameRate, int slot);
    public void setColorFlow(int startIndex, int endIndex, int r, int g, int b, double frameRate, int slot);
    public void clearAnimation(int slot);
    public void setOff();
}