
package frc.robot;

public final class ShooterLookup {

  public static final class Point {
    public final double distM;
    public final double rpm;
    public final double pitchRot; 

    public Point(double distM, double rpm, double pitchRot) {
      this.distM = distM;
      this.rpm = rpm;
      this.pitchRot = pitchRot;
    }
  }


  private static final Point[] TABLE = new Point[] {
      new Point(8.0, 3650.0, -0.40),
      new Point(10.0, 3800.0, -0.44),
      new Point(12.0, 4000.0, -0.62),
      new Point(14.0, 4150.0, -0.67),
      new Point(16.0, 4400.0, -0.70)
  };
  //  private static final Point[] TABLE = new Point[] {
  //     new Point(8.0, 0, -0.20),
  //     new Point(10.0, 0, -0.28),
  //     new Point(12.0, 0, -0.36),
  //     new Point(14.0, 0, -0.40),
  //     new Point(16.0, 0, -0.44)
  // };

  private ShooterLookup() {}

  public static Point sample(double distM) {


    if (!Double.isFinite(distM)) {
      log("BAD DIST", distM, TABLE[0]);
      return TABLE[0];
    }

  
    if (distM <= TABLE[0].distM) {
      log("LOW CLAMP", distM, TABLE[0]);
      return TABLE[0];
    }

    int last = TABLE.length - 1;


    if (distM >= TABLE[last].distM) {
      log("HIGH CLAMP", distM, TABLE[last]);
      return TABLE[last];
    }


    for (int i = 0; i < last; i++) {
      Point a = TABLE[i];
      Point b = TABLE[i + 1];

      if (distM >= a.distM && distM <= b.distM) {

        double t = (distM - a.distM) / (b.distM - a.distM);

        double rpm = lerp(a.rpm, b.rpm, t);
        double pitch = lerp(a.pitchRot, b.pitchRot, t);

        Point result = new Point(distM, rpm, pitch);

        log("INTERP", distM, result);

        return result;
      }
    }

    
    log("FALLBACK", distM, TABLE[last]);
    return TABLE[last];
  }

  private static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private static void log(String tag, double dist, Point p) {
    System.out.printf(
        "[ShooterLookup] %s | dist=%.2f | rpm=%.1f | pitch=%.3f%n",
        tag, dist, p.rpm, p.pitchRot
    );
  }
}