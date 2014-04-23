package map;

import java.util.*;
import java.awt.geom.*;
import java.io.Serializable;

/**
 * <p>Simple configuration space.</p>
 *
 * <p>Each real obstacle is expanded to a CS obstacle by computing the convex
 * hull of the Minkowski sum of the real obstacle and a square circumscribed on
 * the robot bounding disc.</p>
 *
 * @author vona
 **/
public class CSpace implements java.io.Serializable{

  /**
   * <p>The CS obstacles.</p>
   **/
  protected LinkedList<PolygonObstacle> obstacles =
    new LinkedList<PolygonObstacle>();
  
  double a = 0.22;		// x distance from origin to right side
  double b = 0.14;		// y distance from origin to front of robot
  double c = -0.35;		// y distance from origin to back of robot (negative number);

  /**
   * <p>Compute a new CSpace.</p>
   *
   * @param realObstacles the set of real obstacles
   * @param robotRadius the robot disc radius
   **/
  public CSpace(List<PolygonObstacle> realObstacles, double robotRadius) {
    for (PolygonObstacle realObstacle : realObstacles)
      obstacles.add(makeCSObstacle(realObstacle, robotRadius));
  }

  /**
   * <p>Get {@link #obstacles}.</p>
   *
   * @return a reference to <code>obstacles</code>
   **/
  public List<PolygonObstacle> getObstacles() {
    return obstacles;
  }

  /**
   * <p>Make a CS obstacle.</p>
   *
   * @param realObstacle the corresp real obstacle
   * @param robotRadius the robot bounding disc radius (m)
   **/
  protected PolygonObstacle makeCSObstacle(PolygonObstacle realObstacle,
                                           double robotRadius) {

    List<Point2D.Double> csoPoints = new LinkedList<Point2D.Double>();

    List<Point2D.Double> roVertices = realObstacle.getVertices();

    for (Point2D.Double roVertex : roVertices) {
      csoPoints.add(new Point2D.Double(roVertex.x + robotRadius,
                                       roVertex.y + robotRadius));
      csoPoints.add(new Point2D.Double(roVertex.x - robotRadius,
                                       roVertex.y + robotRadius));
      csoPoints.add(new Point2D.Double(roVertex.x - robotRadius,
                                       roVertex.y - robotRadius));
      csoPoints.add(new Point2D.Double(roVertex.x + robotRadius,
                                       roVertex.y - robotRadius));
    }

    PolygonObstacle ret = GeomUtils.convexHull(csoPoints);
    ret.color = realObstacle.color;
    return ret;
  }
  
  /**
   * Make a CS obstacle taking robot rotation into account
   * Theta is passed in radians
   */
  protected PolygonObstacle makeCSObstacleTheta(PolygonObstacle realObstacle, double theta) {
	  List<Point2D.Double> csoPoints = new LinkedList<Point2D.Double>();
	  List<Point2D.Double> roVertices = realObstacle.getVertices();
	  
	  // Determine the robot coordinates with theta rotation in radians (A,B,C,D are CCW starting from robot top right)
	  Point2D.Double A = new Point2D.Double(a*Math.cos(theta) - b*Math.sin(theta), a*Math.sin(theta) + b*Math.cos(theta));
	  Point2D.Double B = new Point2D.Double(-a*Math.cos(theta) - b*Math.sin(theta), -a*Math.sin(theta) + b*Math.cos(theta));
	  Point2D.Double C = new Point2D.Double(-a*Math.cos(theta) - c*Math.sin(theta), -a*Math.sin(theta) + c*Math.cos(theta));
	  Point2D.Double D = new Point2D.Double(a*Math.cos(theta) - c*Math.sin(theta), a*Math.sin(theta) + c*Math.cos(theta));	  
	  
	  // Compute Minkowski sum of reflected robot vertices with real obstacle polygon
	  for (Point2D.Double roVertex : roVertices) {
		  csoPoints.add(new Point2D.Double(roVertex.x - A.x, roVertex.y - A.y));
		  csoPoints.add(new Point2D.Double(roVertex.x - B.x, roVertex.y - B.y));
		  csoPoints.add(new Point2D.Double(roVertex.x - C.x, roVertex.y - C.y));
		  csoPoints.add(new Point2D.Double(roVertex.x - D.x, roVertex.y - D.y));
	  }
	  
	  PolygonObstacle ret = GeomUtils.convexHull(csoPoints);
	  ret.color = realObstacle.color;
	  return ret;
  }
}

