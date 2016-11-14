import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.awt.BorderLayout;
import java.awt.Color;

import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import java.io.InputStream;
import java.util.Enumeration;

import java.util.prefs.Preferences;

import javax.swing.JFrame;
import javax.swing.WindowConstants;


/*
 * Decoder for Neato XV-11's LIDAR Data
 * 
 * http://www.ros.org/wiki/xv_11_laser_driver/Tutorials/Connecting%20the%20XV-11%20Laser%20to%20USB
 * http://labviewhacker.com/doku.php?id=projects:lv_neatolds_interface:lv_neatolds_interface
 * http://xv11hacking.wikispaces.com/LIDAR+Sensor  !!!
 * 
 * OK
 * Piccolo Laser Distance Scanner
 * Copyright (c) 2009-2010 Neato Robotics, Inc.
 * All Rights Reserved
 * 
 * Loader.V2.1.11383
 * CPU.F2802x/cd00
 * Serial.AAA36210AA-0003350
 * LastCal.[2008311154]
 * Runtime.V2.1.12813
 * #Spin...press ESC 3 times to abort...
 * 
 */

public class NeatoLIDAR3 extends JFrame {
  private transient Preferences prefs = Preferences.userRoot().node(this.getClass().getName());
  private static NeatoLIDAR3    lidar;
  private static String         PORT = "COM3";/* "/dev/tty.usbserial-A900J24M";*/
  private Image                 offScr;
  private Dimension             win, lastWin;
  private Range[]               ranges = new Range[0];
  
  public static class Range {
    int range, quality;
    
    public Range (int range, int quality) {
      this.range = range;
      this.quality = quality;
    }
  }
  
  public NeatoLIDAR3 () {
    setBackground(Color.white);
    setLayout(new BorderLayout(1, 1));
    // Add window close handler
    addWindowListener(new WindowAdapter() {
        public void windowClosing(WindowEvent ev) {
          System.exit(0);
        }
      });
    // Track window resize/move events and save in prefs
    addComponentListener(new ComponentAdapter() {
        public void componentMoved(ComponentEvent ev) {
          Rectangle bounds = ev.getComponent().getBounds();
          prefs.putInt("window.x", bounds.x);
          prefs.putInt("window.y", bounds.y);
        }

        public void componentResized(ComponentEvent ev) {
          Rectangle bounds = ev.getComponent().getBounds();
          prefs.putInt("window.width", bounds.width);
          prefs.putInt("window.height", bounds.height);
        }
      });
    setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
    setSize(prefs.getInt("window.width", 800), prefs.getInt("window.height", 800));
    setLocation(prefs.getInt("window.x", 10), prefs.getInt("window.y", 10));
    setVisible(true);
  }
  
  public void paint (Graphics g) {
    win = getSize();
    if (offScr == null || (lastWin != null && (win.width != lastWin.width || win.height != lastWin.height))) {
      offScr = createImage(win.width, win.height);
    }
    lastWin = win;
    Graphics2D g2 = (Graphics2D)offScr.getGraphics();
    g2.setBackground(getBackground());
    g2.clearRect(0, 0, win.width, win.height);
    g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    // Draw ranges
    g2.setColor(Color.BLACK);
    double PI2 = Math.PI * 2.0;
    for (int ii = 0; ii < ranges.length; ii++) {
      Range range = ranges[ii];
      if (range != null) {
        double rad = PI2 * ((double) ii / ranges.length);
        double scale = .3;
        int x = (int) (Math.sin(rad) * range.range * scale);
        int y = (int) (Math.cos(rad) * range.range * scale);
        g2.fillOval(x + (win.width / 2) - 1, y + (win.height / 2) - 1, 3, 3);
      }
    }
    g.drawImage(offScr, 0, 0, this);
  }
  
  private void setRanges (Range[] ranges) {
    this.ranges = ranges;
    repaint();
  }

  private static int getWord (byte[] data, int idx) {
    return ((int) data[idx] & 0xFF) + (((int) data[idx + 1] << 8) & 0xFF00);
  }
  
  public static void main (String[] args) throws Exception {
    lidar = new NeatoLIDAR3();
    CommPortIdentifier portId = null;
    Enumeration portList = CommPortIdentifier.getPortIdentifiers();
    InputStream in = null;
    while (portList.hasMoreElements()) {
      portId = (CommPortIdentifier)portList.nextElement();
      if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
        String pName = portId.getName();
        if (PORT.equals(pName)) {
          SerialPort port = (SerialPort) portId.open("NeatoLIDAR", 3000);
          port.setSerialPortParams(115200, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
          port.setFlowControlMode(SerialPort.FLOWCONTROL_NONE);
          in = port.getInputStream();
          break;
        }
      }
    }
    if (in != null) {
      Range[] ranges = new Range[360];
      int dCount = 360 * 4 + 2;
      byte[] data = new byte[dCount];
      int state = 0;
      int idx = 0;
      int cc;
      while ((cc = in.read()) >= 0) {
        // Process old, version 2.1 firmware data format (Sparkfun type)
        switch (state) {
        case 0:
          state = cc == 0x5A ? 1 : 0;
          idx = 0;
          break;
        case 1:
          state = cc == 0xA5 ? 2 : 0;
          break;
        case 2:
          state = cc == 0x00 ? 3 : 0;
          break;
        case 3:
          state = cc == 0xC0 ? 4 : 0;
          break;
        case 4:
          data[idx++] = (byte) cc;
          if (idx >= dCount) {
            int speed = getWord(data, 2);
            for (int ii = 2; ii < data.length; ii += 4) {
              int rng = getWord(data,  ii);
              int sig = getWord(data,  ii + 2);
              ranges[(ii - 2) / 4] = new Range(rng, sig);
            }
            lidar.setRanges(ranges);
            state = 0;
          }
          break;
        }
      }
    } else {
      System.out.println("Error: port " + PORT + " not found");
    }
  }
}
