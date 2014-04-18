package map;

//import java.awt.geom.Point2D.Double;
import java.lang.Double;
import java.lang.Math;
import java.awt.Color;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.text.ParseException;
import java.util.LinkedList;
import java.util.List;
import java.io.Serializable;

/**
 * <p>The 2D {@link #worldRect}, {@link PolygonObstacle} {@link #obstacles},
 * {@link #robotStart}, and {@link #robotGoal} that
 * make-up the environment in which the robot will navigate.</p>
 *
 * <p>You can either make an instance of this class and use it from within your
 * own program (typical usage), or you can run this code as an indepedent
 * process (see {@link #main}).  The latter mode allows you to display the
 * contents of a map file, but does not give programmatic access to those
 * contents, and thus is useful mostly for displaying/debugging map
 * files.</p>.
 *
 * <p>The format of the map file is as follows (all items are signed doubles in
 * ASCII text format, all units are meters, and all separators are whitespace):
 * <pre>
 * robotStartX robotStartY
 * robotGoalX robotGoalY
 * worldX worldY worldWidth worldHeight
 * obstacle0X0 obstacle0Y0 obstacle0X1 obstacle0Y1 ...
 * ...
 * </pre>
 * The first three lines initialize the corresponding instance fields {@link #robotStart}, {@link #robotGoal}, and {@link
 * #worldRect}, respectively.  Each obstacle line gives the coordinates of the
 * obstacle's vertices in CCW order.  There may be zero obstacles.</p>
 *
 * @author Marty Vona
 * @author Aisha Walcott
 * 
 * @author Kenny Donahue (minor edits 03/11/11)
 * @author Dylan Hadfield-Menell (port to ROS 01/12)
 *
 * @author Bianca Homberg -- changes for rss competition team 6
 **/
public class PolygonMap implements java.io.Serializable{

    // The start point for the robot origin, read in from the map file
    public Point2D.Double robotStart = new Point2D.Double();

    // The goal point for the robot origin, read in from the map file 
    public Point2D.Double robotGoal = new Point2D.Double();

    // The location and size of the world boundary, read in from the map file
    public Rectangle2D.Double worldRect = new Rectangle2D.Double();

    private double ROBOT_RADIUS = .3;

    // The obstacles (does not include the world boundary).</p>
    public LinkedList<PolygonObstacle> obstacles =
	new LinkedList<PolygonObstacle>();

    
    // only starting with two sonars for testing
    private Point2D.Double[] sonarPositions = {new Point2D.Double(.09, 0),
					       new Point2D.Double(-.32, 0)}; 

	//switched to meters
	/*new Point2D.Double(.09, 0),
					       new Point2D.Double(0, -.20),
					       new Point2D.Double(-.32, 0),
					       new Point2D.Double(0, .21)};*/

    private Point2D.Double[] bumpPositions = {new Point2D.Double(10, 10),
					      new Point2D.Double(10, -10)};

    private final double BUMP_THRESHOLD = .1;
    private final double SONAR_MAX_DIST = 2; //pulled out of a hat!!
    private final double SONAR_MIN_DIST = .02; //pulled out of a hat!!
    
    /**
     * <p>Create a new map, parsing <code>mapFile</code>.</p>
     *
     * @param mapFile the map file to parse, or null if none
     **/
    public PolygonMap(File mapFile) throws IOException, ParseException {
	if (mapFile != null)
	    parse(mapFile);
    }


    /**
     * <p>Covers {@link #PolygonMap(File)}.</p>
     **/
    public PolygonMap(String mapFile) throws IOException, ParseException {
	this((mapFile != null) ? new File(mapFile) : null);
    }

    /**
     * <p>Create a new un-initialized polygon map.</p>
     *
     * <p>You may populate it using the accessors below.</p>
     **/
    public PolygonMap() {
    }

    public boolean isValid(double x, double y){
	Point2D.Double pt = new Point2D.Double(x,y);
	for(PolygonObstacle o : obstacles){
	    if(isPointInObstacle(pt, o))
		return false;
	}
	if(x < worldRect.getX() + ROBOT_RADIUS || x > worldRect.getWidth() + worldRect.getX() - .3)
	    return false;
	if(y < worldRect.getY() + ROBOT_RADIUS || y > worldRect.getHeight() + worldRect.getY() - .3)
	    return false;
	return true;
    }

    private boolean isPointInObstacle(Point2D.Double pt, PolygonObstacle po) {
        // no one tell Tej about this code
        double midx = pt.getX();
        double midy = pt.getY();
        double delta = 0.01;// 1 cm offset
        return (po.contains(midx - delta, midy - delta)
                && po.contains(midx - delta, midy + delta)
                && po.contains(midx + delta, midy - delta) && po.contains(midx
                + delta, midy + delta));
    }

    // calculates what the mean sonar values should be given a robot position in the map
    // takes in the robot position
    // returns an array of sonar values
    public double[] predictSonars(double x, double y, double theta){
	double[] rtrn = new double[sonarPositions.length];
	
	for(int i=0; i<sonarPositions.length; i++){
	    // predict a value past the end of the board
	    Point2D.Double sonar_start = localToGlobal(x, y, theta, sonarPositions[i]);
	    //ehhh we should probably make the line below slightly less hacky
	    Point2D.Double sonar_end = localToGlobal(x, y, theta, 
						     new Point2D.Double(sonarPositions[i].getX()*100,
									sonarPositions[i].getY()*100));

	    System.out.println("\n\nPredict sonars: \nX, y, theta: " + x + ", " + y + ", " + theta + "\nsonar start, end: " + sonar_start + ", " + sonar_end);
	    
	    // iterate through obstacles
	    for(PolygonObstacle o : obstacles){
		// iterate through lines on obstacles
		List<Point2D.Double> vertices = o.getVertices();
		for(int j=0; j<vertices.size(); j++){
		    Point2D.Double obs_start = vertices.get(j);
		    Point2D.Double obs_end = vertices.get( (j+1) % vertices.size() );
		    // check for intersection
		    Point2D.Double intersection = getIntersection(sonar_start, sonar_end, obs_start, obs_end);
		    System.out.println("intersection: " + intersection);
		    // if intersection, set the point to the intersection point
		    if(intersection != null)
			if(intersection.getX() != 999 && intersection.getY() != 999)
			    sonar_end = (Point2D.Double)intersection.clone();
		}
	    }
	    
	    System.out.println("new sonar end: " + sonar_end);
	    System.out.println("sonar dist: " + dist(sonar_end, sonar_start) + "\n");
	    // if distance > max distance or < min distance, ignore
	    if(dist(sonar_end, sonar_start) < SONAR_MAX_DIST && dist(sonar_end, sonar_start) > SONAR_MIN_DIST)
		rtrn[i] = dist(sonar_end, sonar_start);
	    else
		rtrn[i] = -1;
	}
	return rtrn;
    }
    
    public Point2D.Double getIntersection(Point2D.Double a, Point2D.Double b, Point2D.Double x, Point2D.Double y){
	//for the moment, ignore vertical lines -- will throw divide by 0 error!

	double m1 = (b.getY() - a.getY()) / (b.getX() - a.getX());
	double c1 = a.getY() - m1 * a.getX();
	
	double m2 = (y.getY() - x.getY()) / (y.getX() - x.getX());
	double c2 = x.getY() - m2 * x.getX();

	double xval = (c2 - c1) / (m1 - m2);
	double yval = m1*xval + c1;

	System.out.println("getIntersection: " + a + "\n" + b + "\n" + x + "\n" + y);
	System.out.println("xval, yval: " + xval + " " + yval);

	// check if intersection point is outisde of the bounds of the two segments
	if((xval > a.getX() && xval > b.getX()) || (xval < a.getX() && xval < b.getX()))
	    return new Point2D.Double(999,999);
	if((xval > x.getX() && xval > y.getX()) || (xval < x.getX() && xval < y.getX()))
	    return new Point2D.Double(999,999);
	if((yval > a.getY() && yval > b.getY()) || (yval < a.getY() && yval < b.getY()))
	    return new Point2D.Double(999,999);
	if((yval > x.getY() && yval > y.getY()) || (yval < x.getY() && yval < y.getY()))
	    return new Point2D.Double(999,999);
	if(((Double)xval).isNaN() || ((Double)yval).isNaN())
	    return new Point2D.Double(999,999);

	return new Point2D.Double(xval, yval);
    }

    public double dist(Point2D.Double a, Point2D.Double b){
	return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2));
    }
    
    public double distToLine(Point2D.Double loc, Point2D.Double s, Point2D.Double e){
	return 0;
    }


    public boolean withinBumpThreshold(double x, double y, double theta, int bumpID){
	Point2D.Double loc = bumpPositions[bumpID];
	loc = localToGlobal(x, y, theta, loc); 

	//want to check if there's an obstacle around loc w/in the threshold

	// iterate through obstacles
	for(PolygonObstacle o : obstacles){
	    // iterate through lines on obstacles
	    List<Point2D.Double> vertices = o.getVertices();
	    for(int j=0; j<vertices.size(); j++){
		Point2D.Double obs_start = vertices.get(j);
		Point2D.Double obs_end = vertices.get( (j+1) % vertices.size() );
		
		double dist = distToLine(loc, obs_start, obs_end);
		if(dist <= BUMP_THRESHOLD){
		    return true;
		}
	    }
	}
	
	// none of the edges were close enough
	return false;
    }

    public Point2D.Double predictFiducial(double x, double y, double theta){
	//TODO: fill in
	return new Point2D.Double(0,0);
    }

    private Point2D.Double localToGlobal(double x, double y, double theta, Point2D.Double loc){
	double xpos = x + loc.getX() * Math.cos(theta) - loc.getY() * Math.sin(theta);
	double ypos = y + loc.getX() * Math.sin(theta) + loc.getY() * Math.cos(theta);
	return new Point2D.Double(xpos, ypos);
    }


    /**
     * <p>Parse a <code>double</code>.</p>
     *
     * @param br the double is expected to be on the next line of this reader
     * @param name the name of the double
     * @param lineNumber the line number of the double
     *
     * @return the parsed double
     *
     * @exception IOException if there was an I/O error
     * @exception ParseException if there was a format error
     * @exception NumberFormatException if there was a format error
     **/
    protected double parseDouble(BufferedReader br, String name, int lineNumber)
	throws IOException, ParseException, NumberFormatException {

	String line = br.readLine();
	if (line == null)
	    throw new ParseException(name + " (line " + lineNumber + ")",
				     lineNumber);

	return Double.parseDouble(line);
    }

    /**
     * <p>Parse a <code>Point2D.Double</code>.</p>
     *
     * @param point the returned point
     * @param br the point is expected to be on the next line of this reader
     * @param name the name of the point
     * @param lineNumber the line number of the point
     *
     * @exception IOException if there was an I/O error
     * @exception ParseException if there was a format error
     * @exception NumberFormatException if there was a format error
     **/
    protected void parsePoint(Point2D.Double point,
			      BufferedReader br, String name, int lineNumber)
	throws IOException, ParseException, NumberFormatException {

	String line = br.readLine();
	String[] tok = (line != null) ? line.split("\\s+") : null;

	if ((tok == null) || (tok.length < 2)){
	    throw new ParseException(name + " (line " + lineNumber + ")",
				     lineNumber);
	}

	point.x = Double.parseDouble(tok[0]);
	point.y = Double.parseDouble(tok[1]);
    }

    /**
     * <p>Parse a <code>Rectangle2D.Double</code>.</p>
     *
     * @param rect the returned rectangle
     * @param br the rect is expected to be on the next line of this reader
     * @param name the name of the rect
     * @param lineNumber the line number of the rect
     *
     * @exception IOException if there was an I/O error
     * @exception ParseException if there was a format error
     * @exception NumberFormatException if there was a format error
     **/
    protected void parseRect(Rectangle2D.Double rect,
			     BufferedReader br, String name, int lineNumber)
	throws IOException, ParseException, NumberFormatException {

	String line = br.readLine();
	String[] tok = (line != null) ? line.split("\\s+") : null;

	if ((tok == null) || (tok.length < 4))
	    throw new ParseException(name + " (line " + lineNumber + ")",
				     lineNumber);

	rect.x = Double.parseDouble(tok[0]);
	rect.y = Double.parseDouble(tok[1]);
	rect.width = Double.parseDouble(tok[2]);
	rect.height = Double.parseDouble(tok[3]);
    }

    /**
     * <p>Parse a {@link PolygonObstacle}.</p>
     *
     * @param br the polygon is expected to be on the next line of this reader
     * @param name the name of the polygon
     * @param lineNumber the line number of the polygon
     *
     * @return a new polygon containing the vertices on the line, or null if
     * there was no line
     *
     * @exception IOException if there was an I/O error
     * @exception ParseException if there was a format error
     * @exception NumberFormatException if there was a format error
     **/
    protected PolygonObstacle parseObs(BufferedReader br,
				       String name, int lineNumber)
	throws IOException, ParseException, NumberFormatException {

	String line = br.readLine();

	if (line == null)
	    return null;

	String[] tok = line.trim().split("\\s+");

	if (tok.length == 0)
	    return null;

	//    System.err.println(
	//      "line " + lineNumber + " (" + tok.length + " tokens): ");
	//    for (int i = 0; i < tok.length; i++)
	//      System.err.println("  " + tok[i]);

	if (tok.length%2 != 0)
	    throw new ParseException(name + " (line " + lineNumber + ")",
				     lineNumber);

	PolygonObstacle poly = new PolygonObstacle();

	for (int i = 0; i < tok.length/2; i++)
	    poly.addVertex(Double.parseDouble(tok[2*i]),
			   Double.parseDouble(tok[2*i+1]));

	poly.close();

	return poly;
    }

    /**
     * <p>Hook called from {@link #parse} after the first four lines are parsed
     * to allow subclasses to parse extra lines before the obstacles.</p>
     *
     * <p>Default impl just returns false.</p>
     *
     * @param br the next line of this reader is the next to be parsed
     * @param lineNumber the line number of the next line of br
     *
     * @return true if the line was parsed and more are expected, false if no
     * more extra lines are expected
     *
     * @exception IOException if there was an I/O error
     * @exception ParseException if there was a format error
     * @exception NumberFormatException if there was a format error
     **/
    protected boolean parseExtra(BufferedReader br, int lineNumber)
	throws IOException, ParseException, NumberFormatException {
	return false;
    }

    /**
     * <p>Parse <code>mapFile</code>.</p>
     *
     * <p>Format is specified in the class header doc for {@link PolygonMap}.</p>
     *
     * @param mapFile the map file, not null
     **/
    protected void parse(File mapFile) throws IOException, ParseException {

	int lineNumber = 1;
	try {

	    BufferedReader br = new BufferedReader(new FileReader(mapFile));

	    parsePoint(robotStart, br, "robot start", lineNumber++);
	    parsePoint(robotGoal, br, "robot goal", lineNumber++);
	    parseRect(worldRect, br, "world rect", lineNumber++);

	    while (parseExtra(br, lineNumber++))
		;

	    for (int obstacleNumber = 0; ; obstacleNumber++) {

		PolygonObstacle poly = parseObs(br, "obstacle " + obstacleNumber,
						lineNumber++);
		if (poly != null) {
		    //poly.color = SonarGUI.makeRandomColor();
		    poly.color = Color.RED;
		    obstacles.add(poly);
		}
		else
		    break;
	    }

	} catch (NumberFormatException e) {
	    throw new ParseException("malformed number on line " + lineNumber,
				     lineNumber);
	}
    }

    /**
     * <p>Get {@link #robotStart}.</p>
     *
     * @return a reference to <code>robotStart</code> (you may modify it)
     **/
    public Point2D.Double getRobotStart() {
	return robotStart;
    }

    /**
     * <p>Get {@link #robotGoal}.</p>
     *
     * @return a reference to <code>robotGoal</code> (you may modify it)
     **/
    public Point2D.Double getRobotGoal() {
	return robotGoal;
    }

    /**
     * <p>Get {@link #worldRect}.</p>
     *
     * @return a reference to <code>worldRect</code> (you may modify it)
     **/
    public Rectangle2D.Double getWorldRect() {
	return worldRect;
    }

    /**
     * <p>Get {@link #obstacles}.</p>
     *
     * @return a reference to <code>obstacles</code> (you may modify it)
     **/
    public List<PolygonObstacle> getObstacles() {
	return obstacles;
    }

    /**
     * <p>Return a human-readable string representation of this map.</p>
     *
     * @return a human-readable string representation of this map
     **/
    @Override public String toString() {

	StringBuffer sb = new StringBuffer();

	sb.append("\nrobot start: ");
	sb.append(Double.toString(robotStart.x));
	sb.append(", ");
	sb.append(Double.toString(robotStart.y));

	sb.append("\nrobot goal: ");
	sb.append(Double.toString(robotGoal.x));
	sb.append(", ");
	sb.append(Double.toString(robotGoal.y));

	sb.append("\nworld rect: x=");
	sb.append(Double.toString(worldRect.x));
	sb.append(" y=");
	sb.append(Double.toString(worldRect.y));
	sb.append(" width=");
	sb.append(Double.toString(worldRect.width));
	sb.append(" height=");
	sb.append(Double.toString(worldRect.height));

	sb.append("\n" + obstacles.size() + " obstacles:");
	for (PolygonObstacle obstacle : obstacles) {
	    sb.append("\n ");
	    obstacle.toStringBuffer(sb);
	}

	return sb.toString();
    }
}
