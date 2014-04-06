package localization;

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
public class PolygonMap{

    // The start point for the robot origin, read in from the map file
    public Point2D.Double robotStart = new Point2D.Double();

    // The goal point for the robot origin, read in from the map file 
    public Point2D.Double robotGoal = new Point2D.Double();

    // The location and size of the world boundary, read in from the map file
    public Rectangle2D.Double worldRect = new Rectangle2D.Double();

    // The obstacles (does not include the world boundary).</p>
    public LinkedList<PolygonObstacle> obstacles =
	new LinkedList<PolygonObstacle>();

    private String mapFile = "/home/rss-staff/ros/rss/solutions/lab6/src/global-nav-maze-2011-basic.map";


    
    /*    public PolygonMap(PolygonMap startMap){
	this.robotStart = startMap.robotStart.clone();
	this.robotGoal = startMap.robotGoal.clone();
	this.worldRect = startMap.worldRect.clone();
	this.obstacles =

	rather than deal with copying all the things, I think it'll be easier to just reparse?
	}*/
        
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


    // calculates what the mean sonar values should be given a robot position in the map
    // takes in the robot position
    // returns an array of sonar values
    public double[] predictSonars(double x, double y, double theta){
	double[] rtrn = new double[4];

	return rtrn;
	// TODO: fill in predictSonars
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
