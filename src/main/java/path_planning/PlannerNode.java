package path_planning;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import rss_msgs.MapMsg;
import rss_msgs.PositionMsg;
import rss_msgs.PositionTargetMsg;
import rss_msgs.WaypointMsg;
import gui_msgs.GUIRectMsg;
import gui_msgs.GUIPolyMsg;
import gui_msgs.GUISegmentMsg;
import gui_msgs.GUIPointMsg;
import gui_msgs.ColorMsg;
import gui.SonarGUI;
import map.CSpace;
import map.PolygonMap;
import map.PolygonObstacle;

public class PlannerNode extends AbstractNodeMain {

    // TODO: Fill me in
    private static final double ROBOT_RADIUS = 0.3;
    private static final int RRT_MAX_POINTS = 300;

    /* Publishers and subscribers */
    private Publisher<WaypointMsg> targetPub;
    private Publisher<GUIRectMsg> guiRectMsgPub;
    private Publisher<GUIPolyMsg> guiPolyMsgPub;
    private Publisher<GUISegmentMsg> guiSegmentMsgPub;
    private Publisher<GUIPointMsg> guiPointMsgPub;
    private Subscriber<PositionTargetMsg> goalSub;
    private Subscriber<PositionMsg> positionSub;
    private Subscriber<MapMsg> mapSub;

    /* Our current pose state */
    private double x;
    private double y;
    private double theta;

    /* Our current map state */
    private PolygonMap map;

    /* Our current waypoints */
    private List<Point2D.Double> waypoints;

    /* Our current rrt graph */
    private RRTStar rrtGraph;

    /* keep track of whether we have a map yet */
    boolean initialized;

    private void handlePositionMsg(PositionMsg msg) {
        // Save our pose
        x = msg.getX();
        y = msg.getY();
        theta = msg.getTheta();
    }

    private void handleGoalMsg(PositionTargetMsg msg) {
        // Update our path plan, and publish to targetPub
	System.out.println(String.format("Got goal message: %f, %f, %f", msg.getX(), msg.getY(), msg.getTheta()));
	if(initialized){
	    System.out.println("Computing RRT path");
	    CSpace cSpace = new CSpace(map.getObstacles(), ROBOT_RADIUS);
	    Point2D.Double start = new Point2D.Double(x, y);
	    Point2D.Double goal = new Point2D.Double(msg.getX(), msg.getY());
	    System.out.println("Before computation");
	    synchronized(this) {
		System.out.println("In synch");
		rrtGraph = new RRTStar(start, goal, map.getWorldRect(), cSpace, RRT_MAX_POINTS);
	    }
	    System.out.println("RRT graph: " + rrtGraph.graph);
	    System.out.println("Map rect: " + map.getWorldRect());
	    waypoints = rrtGraph.computeShortestPath(start, goal);
	    
	    // TODO: Draw full path to gui
	    // Maybe also computed tree? (might be slow)
	    if(waypoints == null){
		System.out.println("No waypoints :(");
		// going forward we're going to need a way to indicate this to state machine
		// it is possible that some areas of the map cannot be traversed to because we're too fat or something
	    }
	    else{
		System.out.println("Got waypoints!! :)");
		Point2D.Double nextWaypoint = waypoints.get(0);
		WaypointMsg waypointMsg = targetPub.newMessage();
		waypointMsg.setX(nextWaypoint.x);
		waypointMsg.setY(nextWaypoint.y);
		waypointMsg.setTheta(-1); //temporarily
		targetPub.publish(waypointMsg);
	    }
	}
    }

    private void handleMapMsg(MapMsg msg) {
        try {
            byte[] ba = msg.getSerializedMap().array();
            ByteArrayInputStream byteStream = new ByteArrayInputStream(ba);
            // Skip the 4-byte length header
            byteStream.skip(4);
            
            ObjectInputStream stream = new ObjectInputStream(byteStream);

            map = (PolygonMap) stream.readObject();
            stream.close();
	    System.out.println("Initialized");
            initialized = true;
        }
        catch (IOException e) {
	    throw new RuntimeException ("IOException in handleMapMsg");
            //e.printStackTrace();
            //return;
        }
        catch (ClassNotFoundException e) {
	    throw new RuntimeException ("ClassNotFoundException in handleMapMsg");
            //e.printStackTrace();
            //return;
        }
    }
        

    @Override
    public void onStart(ConnectedNode node) {

        positionSub = node.newSubscriber("/loc/Position", "rss_msgs/PositionMsg");
        positionSub.addMessageListener(new MessageListener<PositionMsg>() {
            @Override
            public void onNewMessage(PositionMsg msg) {
                handlePositionMsg(msg);
            }
        });
        goalSub = node.newSubscriber("/state/PositionTarget", "rss_msgs/PositionTargetMsg");
        goalSub.addMessageListener(new MessageListener<PositionTargetMsg>() {
            @Override
            public void onNewMessage(PositionTargetMsg msg) {
                handleGoalMsg(msg);
            }
        });
        mapSub = node.newSubscriber("/loc/Map", "rss_msgs/MapMsg");
        mapSub.addMessageListener(new MessageListener<MapMsg>() {
            @Override
            public void onNewMessage(MapMsg msg) {
                handleMapMsg(msg);
            }
        });
        targetPub = node.newPublisher("/path/Waypoint", "rss_msgs/WaypointMsg");
        guiRectMsgPub = node.newPublisher("/gui/Rect", "gui_msgs/GUIRectMsg");
        guiPolyMsgPub = node.newPublisher("/gui/Poly", "gui_msgs/GUIPolyMsg");
        guiSegmentMsgPub = node.newPublisher("/gui/Segment", "gui_msgs/GUISegmentMsg");
        guiPointMsgPub = node.newPublisher("/gui/Point", "gui_msgs/GUIPointMsg");

	initialized = false;

	new DrawThread().start();
    }

    private class DrawThread extends Thread {
        @Override
        public void run() {
            while (true) {
		if (map != null) {
		    System.out.println("Drawing map");
		    displayMap();
		    displayRRTGraph();
		    displayWaypoints();
		}
		else {
		    System.out.println("Map is still null");
		}
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public static void fillRectMsg(GUIRectMsg rectMsg,
            Rectangle2D.Double rect, Color c, boolean filled) {
        rectMsg.getC().setR(c.getRed());
        rectMsg.getC().setG(c.getGreen());
        rectMsg.getC().setB(c.getBlue());
        rectMsg.setX((float) rect.getX());
        rectMsg.setY((float) rect.getY());
        rectMsg.setWidth((float) rect.getWidth());
        rectMsg.setHeight((float) rect.getHeight());
        rectMsg.setFilled(filled ? 1 : 0);
    }
    
    public static void fillPolyMsg(GUIPolyMsg polyMsg, PolygonObstacle obs,
            Color c, boolean filled, boolean closed) {
        polyMsg.getC().setR(c.getRed());
        polyMsg.getC().setG(c.getGreen());
        polyMsg.getC().setB(c.getBlue());
        List<Point2D.Double> points = obs.getVertices();
        float[] x = new float[points.size()];
        float[] y = new float[points.size()];
        for (int i = 0; i < points.size(); i++) {
            x[i] = (float) points.get(i).x;
            y[i] = (float) points.get(i).y;
        }
        polyMsg.setX(x);
        polyMsg.setY(y);
        polyMsg.setClosed(closed ? 1 : 0);
        polyMsg.setFilled(filled ? 1 : 0);
        polyMsg.setNumVertices(points.size());
    }
    
    public static void fillPointMsg(GUIPointMsg pointMsg,
            Point2D.Double point, Color color, long shape) {
        pointMsg.setX(point.x);
        pointMsg.setY(point.y);
        pointMsg.setShape(shape);
        pointMsg.getColor().setR(color.getRed());
        pointMsg.getColor().setG(color.getGreen());
        pointMsg.getColor().setB(color.getBlue());
    }
    
    public static void fillSegmentMsg(GUISegmentMsg segMsg,
            Point2D.Double start, Point2D.Double end, Color color) {
        segMsg.setStartX(start.x);
        segMsg.setStartY(start.y);
        segMsg.setEndX(end.x);
        segMsg.setEndY(end.y);
        segMsg.getColor().setR(color.getRed());
        segMsg.getColor().setG(color.getGreen());
        segMsg.getColor().setB(color.getBlue());
    }

    /**
     * Display the map inputed in the onStart method to MapGUI.
     */
    protected void displayMap() {
        // Draw the world rectangle
        GUIRectMsg rectMsg = guiRectMsgPub.newMessage();
        fillRectMsg(rectMsg, map.getWorldRect(), Color.GRAY, false);
        guiRectMsgPub.publish(rectMsg);
        
        // Draw obstacle polygons
        List<PolygonObstacle> obstacles = map.getObstacles();
        for (PolygonObstacle obs : obstacles) {
            GUIPolyMsg polyMsg = guiPolyMsgPub.newMessage();
            fillPolyMsg(polyMsg, obs, Color.BLUE, true, obs.isClosed());
            guiPolyMsgPub.publish(polyMsg);
        }
        
        // Draw robot start and end
        Point2D.Double robotStart = map.getRobotStart();
        Point2D.Double robotGoal = map.getRobotGoal();
        
        GUIPointMsg start = guiPointMsgPub.newMessage();
        fillPointMsg(start, robotStart, Color.RED, SonarGUI.O_POINT);
        guiPointMsgPub.publish(start);
        
        GUIPointMsg goal = guiPointMsgPub.newMessage();
        fillPointMsg(goal, robotGoal, Color.GREEN, SonarGUI.X_POINT);
        guiPointMsgPub.publish(goal);
    }

    /**
     * Draw the RRT graph to the MapGUI.
     */
    protected void displayRRTGraph() {
	if (rrtGraph == null) return;
	Map<Point2D.Double, ArrayList<Point2D.Double>> graph;
	synchronized(this) {
	    graph = rrtGraph.graph;
	}
	for (Point2D.Double first : graph.keySet()) {
	    for (Point2D.Double second : graph.get(first)) {
		GUISegmentMsg segMsg = guiSegmentMsgPub.newMessage();
		fillSegmentMsg(segMsg, first, second, Color.GREEN);
		guiSegmentMsgPub.publish(segMsg);
	    }
	}	
    }

    /**
     * Draw a path of the current waypoints to the MapGUI.
     */
    protected void displayWaypoints() {
        if (waypoints == null || waypoints.size() < 2) {
	    System.out.println("No waypoints: " + waypoints);
	    return;
	}
	Object[] pointsArray = waypoints.toArray();
        
        for (int i = 1; i < waypoints.size(); i++) {
            Point2D.Double start = (Point2D.Double) pointsArray[i-1];
            Point2D.Double end = (Point2D.Double) pointsArray[i];
            GUISegmentMsg segMsg = guiSegmentMsgPub.newMessage();
            fillSegmentMsg(segMsg, start, end, Color.RED);
            guiSegmentMsgPub.publish(segMsg);
        }
    }


    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("path_planner");
    }
}
