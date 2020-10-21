package roadgraph;

import geography.GeographicPoint;
import java.util.*;

// A class which represents individual nodes in the graph

public class MapNode implements Comparable<MapNode>{
	
	private GeographicPoint location;
	private HashMap<MapEdge, GeographicPoint> neighbours;
	private double timeToStart;
	private double predictedTime;
	private double distanceToStart;
	private double predictedDistance;
	
	/**
	 * Constructor for MapNode class
	 * @param addLocation Location of current node
	 */
	public MapNode(GeographicPoint addLocation) {
		this.location = addLocation;
		this.distanceToStart = Double.POSITIVE_INFINITY;
		this.predictedDistance = Double.POSITIVE_INFINITY;
		this.timeToStart = Double.POSITIVE_INFINITY;
		this.predictedTime = Double.POSITIVE_INFINITY;
		neighbours = new HashMap<MapEdge, GeographicPoint> ();
	}
	
	/**
	 * Adds node to the "neighbours" hash map
	 * @param neighbour Location of neighbour node to be added
	 * @param distance Distance between current node and the node to be added
	 * @param roadName Name of the edge connecting both nodes
	 * @param roadType Type of the edge connecting both nodes
	 * @return True if this neighbour node is successfully added
	 */
	public boolean addNeighbours(GeographicPoint neighbour, double distance, String roadName, String roadType) {
		if (!neighbours.containsValue(neighbour)) {
			MapEdge newEdge = new MapEdge(location, neighbour, distance, roadName, roadType);
			neighbours.put(newEdge, neighbour);
			return true;
		}
		return false;
	}
	
	/**
	 * @return "neighbours" hash map
	 */
	public HashMap<MapEdge, GeographicPoint> getNeighbours() {
		return neighbours;
	}
	
	/**
	 * 
	 * @return location of current node
	 */
	public GeographicPoint getLocation() {
		return location;
	}
	
	public double getTimeToStart() {
		return timeToStart;
	}
	
	public double getPredictedTime() {
		return predictedTime;
	}
	
	public double getDistanceToStart() {
		return distanceToStart;
	}
	
	public double getPredictedDistance() {
		return predictedDistance;
	}
	
	public void setTimeToStart(double other) {
		timeToStart = other;
	}
	
	public void setPredictedTime(double other) {
		predictedTime = other;
	}
	
	public void setPredictedDistance(double other) {
		predictedDistance = other;
	}
	
	public void setDistanceToStart(double other) {
		distanceToStart = other;
	}
	
	public void printAllNeighbours() {
		for (GeographicPoint curr : neighbours.values()) {
			System.out.print(curr.toString() + " ");
		}
		System.out.println();
	}
	
	public String toString() {
		return location.toString();
	}
	
	@Override
	public int compareTo(MapNode o) {
		// TODO Auto-generated method stub
		if (this.timeToStart == o.timeToStart)
			return 0;
		else if (this.timeToStart > o.timeToStart)
			return 1;
		else
			return -1;
//		return (int) (this.distanceToStart - o.distanceToStart);
	}

}
