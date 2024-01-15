// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.livoniawarriors;

import java.util.Objects;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Represents a color in HSV space.
 * 
 * In WPILib, the convention is hue [0-180)*, saturation [0-255], value
 * [0-255].  See https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#using-hsv-values
 * for more info.
 */
public class ColorHSV {
  /** Value between [0-180). */
  public final double hue;

  /** Value between [0-255]. */
  public final double sat;

  /** Value between [0-255]. */
  public final double value;

  /** Constructs a default color (black). */
  public ColorHSV() {
    hue = 0.0;
    sat = 0.0;
    value = 0.0;
  }

  public ColorHSV(double hue, double sat, double value) {
    this.hue = hue;
    this.sat = sat;
    this.value = value;
  }

  public ColorHSV(int hue, int sat, int value) {
    this.hue = hue;
    this.sat = sat;
    this.value = value;
  }

  public static ColorHSV fromColor(Color color) {
    // based on (python code): https://math.stackexchange.com/a/3954976
    double min = Math.min(Math.min(color.red, color.green), color.blue);
    double max = Math.max(Math.max(color.red, color.green), color.blue);
    double sat;
    double hue;
    if (max == 0) {
      sat = 0;
      hue = 0;
    } else {
      sat = ((max - min) / max) * 255.0;
      if (max == color.red) {
        hue = ((color.green - color.blue) / (max - min)) * 30.0;
        if (hue < 0) {
          hue += 180;
        }
      } else if (max == color.green) {
        hue = (((color.blue - color.red) / (max - min)) + 2) * 30.0;
      } else { // blue
        hue = (((color.red - color.green) / (max - min)) + 4) * 30.0;
      }
    }
    return new ColorHSV(hue, sat, max * 255.0);
  }

  public static ColorHSV fromColor8Bit(Color8Bit color) {
    return fromColor(new Color(color));
  }

  //rgb are 0-255
  public static ColorHSV fromRGB(int red, int green, int blue) {
    return fromColor(new Color(red, green, blue));
  }

  //throws IllegalArguementException
  //public static ColorHSV fromHexString(String hexString) {
  //  return fromColor(new Color(hexString));
  //}

  public Color toColor() {
    // Loosely based on
    // https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB
    // The hue range is split into 60 degree regions where in each region there
    // is one rgb component at a low value (m), one at a high value (v) and one
    // that changes (X) from low to high (X+m) or high to low (v-X)

    // Difference between highest and lowest value of any rgb component
    final double chroma = (sat * value) / 255.0;

    // Remainder converted from 0-30 to 0-255
    final double remainder = (hue % 30) * (255 / 30.0);

    // Value of the lowest rgb component
    final double m = (value - chroma) / 255.0;

    // Goes from 0 to chroma as hue increases
    final double X = (chroma * remainder) / (255.0 * 255);

    // need to scale to 0-1 range
    final double V = value / 255.0;

    // Because hue is 0-180 rather than 0-360 use 30 not 60
    final int region = (int)(hue / 30) % 6;
    switch (region) {
      case 0:
        return new Color(V, X + m, m);
      case 1:
        return new Color(V - X, V, m);
      case 2:
        return new Color(m, V, X + m);
      case 3:
        return new Color(m, V - X, V);
      case 4:
        return new Color(X + m, m, V);
      default:
        return new Color(V, m, V - X);
    }
  }

  public Color8Bit toColor8Bit() {
    return new Color8Bit(toColor());
  }

  @Override
  public boolean equals(Object other) {
    if (this == other) {
      return true;
    }
    if (other == null || getClass() != other.getClass()) {
      return false;
    }
    ColorHSV colorHSV = (ColorHSV) other;

    //to keep our 12 bit precision, we use 8 bits for the components, 
    //so 4 bits left for precision (1/2^4 = 1/16 = 0.0625)
    return Math.abs(colorHSV.hue - hue) <= 0.0625 
        && Math.abs(colorHSV.sat - sat) <= 0.0625 
        && Math.abs(colorHSV.value - value) <= 0.0625;
  }

  @Override
  public int hashCode() {
    return Objects.hash(hue, sat, value);
  }

  @Override
  public String toString() {
    return String.format("HSV(%.0f,%.0f,%.0f)", hue, sat, value);
  }
}
