package path_planning;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;

import map.CSpace;
import map.PolygonObstacle;

public class RRTStar {

    private final double EXTENSION_LENGTH = .5;
    private Random rand = new Random();
    private Point2D.Double start;
    private Point2D.Double goal;
    private Rectangle2D.Double cworldRect;
    private CSpace cspace;
    private int maxPoints;
    

    // This is the class used for the A* search for the shortest path
    // Each queue element is a path that has the distance traveled and the
    // distance to the goal
    // They are compared by looking at total distance (distFromStart +
    // distToGoal)
    public class QueueElement implements Comparable<QueueElement> {
        public ArrayList<Point2D.Double> path;
        public double distanceToGoal;
        public double distanceFromStart;

        public QueueElement(ArrayList<Point2D.Double> p, double distFromStart,
                double distToGoal) {
            path = new ArrayList<Point2D.Double>();
            path = p;
            distanceToGoal = distToGoal;
            distanceFromStart = distFromStart;
        }

        @Override
        public int compareTo(QueueElement obj) {
            if (this.distanceToGoal + this.distanceFromStart < obj.distanceToGoal
                    + obj.distanceFromStart) {
                return -1;
            }
            if (this.distanceToGoal + this.distanceFromStart > obj.distanceToGoal
                    + obj.distanceFromStart) {
                return 1;
            }
            return 0;
        }
    }

    public Map<Point2D.Double, ArrayList<Point2D.Double>> graph = new HashMap<Point2D.Double, ArrayList<Point2D.Double>>();
    private Map<Point2D.Double, Point2D.Double> parent = new HashMap<Point2D.Double, Point2D.Double>();
    private Map<Point2D.Double, Double> cost = new HashMap<Point2D.Double, Double>();

    public RRTStar(Point2D.Double start, Point2D.Double goal,
            Rectangle2D.Double cworldRect, CSpace cspace, int maxPoints) {
	this.start = start;
	this.goal = goal;
	this.cworldRect = cworldRect;
	this.cspace = cspace;
	this.maxPoints = maxPoints;
    }

    public void setGoal(Point2D.Double goal){
	this.goal = goal;
    }

    public void setStart(Point2D.Double start){
	this.start = start;
    }

    public void setCworldRect(Rectangle2D.Double cworldRect){
	this.cworldRect = cworldRect;
    }

    public void setCSpace(CSpace cspace){
	this.cspace = cspace;
    }

    public void setMaxPoints(int m){
	this.maxPoints = m;
    }

    public Map<Point2D.Double, ArrayList<Point2D.Double>> compute(){
        /*
         * ArrayList<Point2D.Double> allPoints = new
         * ArrayList<Point2D.Double>(); allPoints.add(start);
         * allPoints.add(goal); for(PolygonObstacle po : cspace.getObstacles()){
         * for(Point2D.Double point : po.getVertices()){ allPoints.add(point); }
         * }
         */
        // start = (Point2D.Double) start;

        // initialize the RRT graph
        graph.put(start, new ArrayList<Point2D.Double>());
        parent.put(start, start);
        cost.put(start, (double) 0);

        // Create the RRT graph
        // we keep iterating until the goal point is reached or until we've
        // added maxPoints
        Point2D.Double beginning, end;
        int count = 0;
        boolean found = false;
        while (count < maxPoints && (!found)) {
	    count++;
            if (rand.nextDouble() < .15)
                end = goal;
            else
                end = getRandomPoint(cworldRect);
            beginning = getClosestPoint(end);
            if (end != goal)
                end = getExtension(beginning, end);
            //System.out.println("End point: " + end);
	    if (canSee(beginning, end, cspace, cworldRect)) {
		//System.out.println("We can see the end point!");
                // TODO: Generate the correct constant for this
                List<Point2D.Double> nearPoints = getNearPoints(end, 2);
		//System.out.println("Near points: " + nearPoints);
                // Find minimum cost path to end from near points.
                double minCost = cost.get(beginning) + euclideanDistance(beginning, end);
                Point2D.Double nearest = beginning;
                for (Point2D.Double nearpt : nearPoints) {
                    if (canSee(nearpt, end, cspace, cworldRect) &&
                        cost.get(nearpt) + euclideanDistance(nearpt, end) < minCost) {
                        minCost = cost.get(nearpt) + euclideanDistance(nearpt, end);
                        nearest = nearpt;
                    }
                }
		ArrayList<Point2D.Double> newPoint = new ArrayList<Point2D.Double>();
		newPoint.add(nearest);
                graph.put(end, newPoint);
                parent.put(end, nearest);
                cost.put(end, minCost);
                graph.get(nearest).add(end);
		

                // Rewire the tree to use min-cost paths that go through the new point
                for (Point2D.Double nearpt : nearPoints) {
                    if (canSee(nearpt, end, cspace, cworldRect) &&
                        cost.get(end) + euclideanDistance(nearpt, end) < cost.get(nearpt)) {
                        // Replace the edge from the parent of nearpt to nearpt
                        graph.get(parent.get(nearpt)).remove(nearpt);
                        graph.get(end).add(nearpt);
                        parent.put(nearpt, end);
                        cost.put(nearpt, cost.get(end) + euclideanDistance(nearpt, end));
                    }
                }

                if (end == goal)
                    found = true;
            }
        }
	return graph;
    }

    public Point2D.Double getRandomPoint(Rectangle2D.Double cworldRect) {
        double x = rand.nextDouble() * cworldRect.getWidth()
                + cworldRect.getX();
        double y = rand.nextDouble() * cworldRect.getHeight()
                + cworldRect.getY();
        return new Point2D.Double(x, y);
    }

    public Point2D.Double getClosestPoint(Point2D.Double pt) {
        Point2D.Double minpt = null;
        double minDist = -1;
        for (Point2D.Double graphpt : graph.keySet()) {
            if (minDist == -1 || euclideanDistance(pt, graphpt) < minDist) {
                minDist = euclideanDistance(pt, graphpt);
                minpt = graphpt;
            }
        }
        return minpt;
    }

    public List<Point2D.Double> getNearPoints(Point2D.Double pt, double radius) {
        List<Point2D.Double> nearPoints = new ArrayList<Point2D.Double>();
        for (Point2D.Double graphpt: graph.keySet()) {
            if (euclideanDistance(pt, graphpt) < radius) {
                nearPoints.add(graphpt);
            }
        }
        return nearPoints;
    }

    public Point2D.Double getExtension(Point2D.Double start, Point2D.Double end) {
        // making all extensions half a meter long...we should check whether
        // this is a reasonable assumption

        if (euclideanDistance(start, end) < EXTENSION_LENGTH)
            return end;

        // we're using atan2 so this should work for all variations of start/end
        // using start as origin
        double theta = Math.atan2(end.getY() - start.getY(),
                end.getX() - start.getX());
        double x = Math.cos(theta) * EXTENSION_LENGTH + start.getX();
        double y = Math.sin(theta) * EXTENSION_LENGTH + start.getY();

        return new Point2D.Double(x, y);
    }

    // Make 4 checks around the midpoint of a line.
    // If all 4 checks fall into the object, then we return true
    // If only a few checks fall in the object, then the line is probably an
    // obstacle edge, which is fine.
    private boolean isMidpointInObstacle(Line2D.Double line, PolygonObstacle po) {
        double midx = (line.getP1().getX() + line.getP2().getX()) / 2.0;
        double midy = (line.getP1().getY() + line.getP2().getY()) / 2.0;
        double delta = 0.01;// 1 cm offset
        return (po.contains(midx - delta, midy - delta)
                && po.contains(midx - delta, midy + delta)
                && po.contains(midx + delta, midy - delta) && po.contains(midx
                + delta, midy + delta));
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

    private boolean canSee(Point2D.Double start, Point2D.Double end,
            CSpace cspace, Rectangle2D.Double cworldRect) {
        Line2D.Double startToEndLine = new Line2D.Double(start, end);
        if (!cworldRect.contains(start) || !cworldRect.contains(end)) {
            return false;
        }
        for (PolygonObstacle po : cspace.getObstacles()) {
            if (isMidpointInObstacle(startToEndLine, po)) {
                return false;
            }
            if (isPointInObstacle(start, po)) {
                return false;
            }
            if (isPointInObstacle(end, po)) {
                return false;
            }
            ArrayList<Line2D.Double> lines = new ArrayList<Line2D.Double>();
            Point2D.Double last = po.getVertices().get(
                    po.getVertices().size() - 1);
            for (Point2D.Double vertex : po.getVertices()) {
                lines.add(new Line2D.Double(last, vertex));
                last = vertex;
            }
            for (Line2D.Double otherLine : lines) {
                if (startToEndLine.intersectsLine(otherLine)
                        && !startToEndLine.equals(otherLine)
                        && !otherLine.getP1().equals(start)
                        && !otherLine.getP2().equals(start)
                        && !otherLine.getP1().equals(end)
                        && !otherLine.getP2().equals(end)) {
                    return false;
                }
            }

        }
        return true;
    }

    // Needed a quick utility
    private double euclideanDistance(Point2D.Double start, Point2D.Double end) {
        return Math.sqrt(Math.pow(start.getX() - end.getX(), 2)
                + Math.pow(start.getY() - end.getY(), 2));
    }

    // A*. visited stores a set of the visited nodes because the first time we
    // visit a node
    // represents the shortest path to that node, so it doesn't make sense to
    // ever visit a node
    // again.
    // The queue stores a list of Queue elements. They are sorted by the A*
    // heuristic (distTraveled+distToGoal)
    // If we pop off the queue and the path ends at the goal, that is our
    // desired path
    // Otherwise, we extend the node at the end of the path, add it to the
    // visited nodes, and add
    // all possible paths from that node to the queue, assuming the next node
    // isn't already in the path or in our visited list
    // If the queue is ever empty, we have no path
    @SuppressWarnings("unchecked")
    public List<Point2D.Double> computeShortestPath(Point2D.Double start,
            Point2D.Double goal) {
        PriorityQueue<QueueElement> queue = new PriorityQueue<QueueElement>();

        Set<Point2D.Double> visited = new HashSet<Point2D.Double>();
        ArrayList<Point2D.Double> initialPath = new ArrayList<Point2D.Double>();
        initialPath.add(start);
        queue.add(new QueueElement(initialPath, 0.0, euclideanDistance(start,
                goal)));

        while (!queue.isEmpty()) {
            QueueElement current = queue.poll();
            Point2D.Double currentPoint = current.path
                    .get(current.path.size() - 1);
            visited.add(currentPoint);
            if (currentPoint.equals(goal)) {
                return current.path;
            }
	    
	    if(graph.get(currentPoint) != null){
		for (Point2D.Double neighbor : graph.get(currentPoint)) {
		    if (visited.contains(neighbor)) {
			continue;
		    }
		    if (current.path.contains(neighbor)) {
			continue;
		    }
		    ArrayList<Point2D.Double> newPath = (ArrayList<Point2D.Double>) current.path
                        .clone();
		    newPath.add(neighbor);
		    queue.add(new QueueElement(newPath, current.distanceFromStart
					       + euclideanDistance(currentPoint, neighbor),
					       euclideanDistance(neighbor, goal)));
		}
	    }
        }
        return null;
    }

    // Make the get so we can render the graph from GlobalNavigation.java
    public Map<Point2D.Double, ArrayList<Point2D.Double>> getGraph() {
        return graph;
    }
}
