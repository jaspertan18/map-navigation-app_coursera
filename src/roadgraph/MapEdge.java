package roadgraph;

import geography.GeographicPoint;

// A class which represents neighbours of each MapNode in the graph
// Stores information such as name and type of edge connecting MapNode to neighbour nodes
// Also stores the distance between node and neighbour nodes

public class MapEdge{
	
	private GeographicPoint goal;
	private GeographicPoint begin;
	private String streetName;
	private String streetType;
	private double distanceToEndNode;
	private double duration;
	
	/**
	 * Constructor for MapEdge class
	 * @param end Location of neighbour node
	 * @param distance Distance between MapNode and neighbour node
	 * @param roadName Name of the edge connecting both nodes
	 * @param roadType Type of the edge connecting both nodes
	 */
	public MapEdge(GeographicPoint start, GeographicPoint end, double distance, String roadName, String roadType) {
		this.goal = end;
		this.begin = start;
		this.streetName = roadName;
		this.streetType = roadType;
		this.distanceToEndNode = distance;
		this.duration = setSpeedLimit(streetType);
	}
	
	private double setSpeedLimit(String roadType) {
		switch (roadType) {
			case "living_street":
				return 5.0;
			case "residential":
				return 5.0;
			case "secondary":
				return 3.0;
			case "tertiary":
				return 1.5;
			case "unclassified":
				return 1.0;
			default:
				return 0.0;
		}
	}
	
	public double getSpeedLimit() {
		return duration;
	}
	
	/**
	 * Getter method for name of edge connecting MapNode and neighbour node
	 * @return Name of the edge
	 */
	public String getStreetName() {
		return streetName;
	}
	
	/**
	 * Getter method for type of edge connecting MapNode and neighbour node
	 * @return Type of the edge
	 */
	public String getStreetType() {
		return streetType;
	}
	
	/**
	 * Getter method for distance between MapNode and neighbour node
	 * @return Distance between both nodes
	 */
	public double getDistance() {
		return distanceToEndNode;
	}
	
	
	/**
	 * Getter method for location of node
	 * @return Location of node
	 */
	public GeographicPoint getEnd() {
		return goal;
	}
	
	public GeographicPoint getStart() {
		return begin;
	}
	
	/**
	 * Setter method for location of node
	 * @param location New location for node
	 * @return True is location is updated
	 */
	public boolean setLocation(GeographicPoint location) {
		if (!goal.equals(location)) {
			goal = location;
			return true;
		}
		return false;
	}
	
	/**
	 * Setter method for type of edge connecting node and neighbour node
	 * @param type New type for the edge
	 * @return True if type is successfully updated
	 */
	public boolean setStreetType(String type) {
		if (!type.equals(streetType) && type.length() > 0) {
			streetType = type;
			return true;
		}
		return false;
	}
	
	/**
	 * Setter method for name of edge connecting node and neighbour node
	 * @param name New name for the edge
	 * @return True if name is successfully updated
	 */
	public boolean setStreetName(String name) {
		if (!name.equals(streetName) && name.length() > 0) {
			streetName = name;
			return true;
		}
		return false;
	}
	
	/**
	 * Setter method for distance between node and neighbour node
	 * @param length New distance to be updated
	 * @return True if distance is successfully updated
	 */
	public boolean setDistance(double length) {
		if (length >= 0 && length != distanceToEndNode) {
			distanceToEndNode = length;
			return true;
		}
		return false;
	}
	
	public String toString() {
		String ans = "Connecting ";
		ans += begin.toString() + " to " + goal.toString();
		return ans;
	}
}
