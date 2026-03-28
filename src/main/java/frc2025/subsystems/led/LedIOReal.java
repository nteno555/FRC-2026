package frc2025.subsystems.led;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import edu.wpi.first.units.Units;
import frc2025.config.SubsystemConstants;

public class LedIOReal implements LedIO {

    private final CANdle candle = new CANdle(111);

    public LedIOReal() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = 1.0;
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        candle.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(LedIOInputs inputs) {
        inputs.candleConnected = candle.isConnected();
        inputs.supplyVoltage = candle.getSupplyVoltage().getValue().in(Units.Volts);
        inputs.outputCurrent = candle.getOutputCurrent().getValue().in(Units.Amps);
        inputs.deviceTempCelsius = candle.getDeviceTemp().getValue().in(Units.Celsius);
    }

    @Override
    public void setSolidColor(int startIndex, int endIndex, int r, int g, int b) {
        candle.setControl(new SolidColor(startIndex, endIndex)
                .withColor(new RGBWColor(r, g, b, 0)));
    }

    @Override
    public void setRainbow(int startIndex, int endIndex, double brightness, double frameRate, int slot) {
        candle.setControl(new RainbowAnimation(startIndex, endIndex)
                .withBrightness(brightness)
                .withFrameRate(frameRate)
                .withSlot(slot));
    }

    @Override
    public void setLarson(int startIndex, int endIndex, int r, int g, int b, int size, double frameRate, int slot) {
        candle.setControl(new LarsonAnimation(startIndex, endIndex)
                .withColor(new RGBWColor(r, g, b, 0))
                .withSize(size)
                .withFrameRate(frameRate)
                .withSlot(slot));
    }

    @Override
    public void setStrobe(int startIndex, int endIndex, int r, int g, int b, double frameRate, int slot) {
        candle.setControl(new StrobeAnimation(startIndex, endIndex)
                .withColor(new RGBWColor(r, g, b, 0))
                .withFrameRate(frameRate)
                .withSlot(slot));
    }

    @Override
    public void setSingleFade(int startIndex, int endIndex, int r, int g, int b, double frameRate, int slot) {
        candle.setControl(new SingleFadeAnimation(startIndex, endIndex)
                .withColor(new RGBWColor(r, g, b, 0))
                .withFrameRate(frameRate)
                .withSlot(slot));
    }

    @Override
    public void setRgbFade(int startIndex, int endIndex, double brightness, double frameRate, int slot) {
        candle.setControl(new RgbFadeAnimation(startIndex, endIndex)
                .withBrightness(brightness)
                .withFrameRate(frameRate)
                .withSlot(slot));
    }

    @Override
    public void setFire(int startIndex, int endIndex, double brightness, double sparking, double cooling, double frameRate, int slot) {
        candle.setControl(new FireAnimation(startIndex, endIndex)
                .withBrightness(brightness)
                .withSparking(sparking)
                .withCooling(cooling)
                .withFrameRate(frameRate)
                .withSlot(slot));
    }

    @Override
    public void setColorFlow(int startIndex, int endIndex, int r, int g, int b, double frameRate, int slot) {
        candle.setControl(new ColorFlowAnimation(startIndex, endIndex)
                .withColor(new RGBWColor(r, g, b, 0))
                .withFrameRate(frameRate)
                .withSlot(slot));
    }

    @Override
    public void clearAnimation(int slot) {
        candle.setControl(new EmptyAnimation(slot));
    }

    @Override
    public void setOff() {
        setSolidColor(0, 399, 0, 0, 0);
    }
}
