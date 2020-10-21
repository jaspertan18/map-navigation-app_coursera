/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.*;
//import java.util.List;
//import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraphExtra {
	//TODO: Add your member variables here in WEEK 3
	private HashMap<GeographicPoint, MapNode> map;
	private int numVertices;
	private int numEdges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraphExtra()
	{
		// TODO: Implement in this constructor in WEEK 3
		map = new HashMap<GeographicPoint, MapNode> ();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		Set<GeographicPoint> ans = new HashSet<GeographicPoint> ();
		for (GeographicPoint curr : map.keySet()) {
			if (!ans.contains(curr)) {
				ans.add(curr);
			}
		}
		return ans;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if (location == null) {
			return false;
		}
		else if (!map.containsKey(location)) {
			map.put(location, new MapNode(location));
			numVertices++;
			return true;
		}
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		
		if (length < 0 || (!map.containsKey(from) || !map.containsKey(to)) || (from == null || to == null)) {
			throw new IllegalArgumentException();
		}
		MapNode start = map.get(from);
		boolean added = start.addNeighbours(to, length, roadName, roadType);
		if (added) {
			numEdges++;
		}
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */

	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		List<GeographicPoint> queue = new ArrayList<GeographicPoint>();
		HashMap<GeographicPoint, List<GeographicPoint>> prev = new HashMap<GeographicPoint, List<GeographicPoint>>();
		
		for (GeographicPoint temp : map.keySet())
			prev.put(temp, new ArrayList<GeographicPoint>());
		
		if (!map.containsKey(start) || !map.containsKey(goal)) 
			return null;
		
		queue.add(start);
		while (!queue.isEmpty() && !visited.contains(goal)) {
			
			GeographicPoint currPoint = queue.get(0);
			MapNode currNode = map.get(currPoint);
			nodeSearched.accept(currPoint);
			
			queue.remove(0);
			HashMap<MapEdge, GeographicPoint> neighbours = currNode.getNeighbours();
			for (GeographicPoint n : neighbours.values()) {
				if (n.equals(start))
					continue;
				if (!visited.contains(n)) {
					queue.add(n);
					visited.add(n);
					prev.get(n).add(currPoint);
				}
			}
		}
		return backTrack(prev, goal);
	}
	
	/** Find the shortest path from start to goal by backtracking from goal
	 * 
	 * @param prev Maps each node to a the node visited right before moving to the current node
	 * @param goal The final point we wish to visit
	 * @return A list of shortest path from the starting point to the end point
	 */
	private List<GeographicPoint> backTrack(HashMap<GeographicPoint, List<GeographicPoint>> prev, GeographicPoint goal) {
		List<GeographicPoint> ans = new ArrayList<GeographicPoint>();
		List<GeographicPoint> temp = prev.get(goal);
		if (temp.size() != 0) {
			ans.add(goal);
			while (temp.size() != 0) {
//				System.out.print("Neighbours: ");
				// lastNeighbour will be the neighbour node we visited right before moving to the current node
				GeographicPoint lastNeighbour = temp.get(temp.size() - 1);
				ans.add(lastNeighbour);
				temp = prev.get(lastNeighbour);
			}
			Collections.reverse(ans);
		}
		if (ans.isEmpty())
			return null;
		return ans;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		//initialize distance of each MapNode from start to infinity
		
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>();
		HashMap<GeographicPoint, List<GeographicPoint>> path = new HashMap<GeographicPoint, List<GeographicPoint>>();
		int count = 0;
		
		if (!map.containsKey(start) || !map.containsKey(goal))
			return null;
		
		for (GeographicPoint temp : map.keySet())
			path.put(temp, new ArrayList<GeographicPoint>());
		
		MapNode startNode = map.get(start);
		startNode.setTimeToStart(0.0);
		startNode.setDistanceToStart(0.0);
		queue.add(startNode);
		
		while (!queue.isEmpty()) {
			MapNode currNode = queue.poll();
			nodeSearched.accept(currNode.getLocation());
			
			if (!visited.contains(currNode.getLocation())) {
				visited.add(currNode.getLocation());
				count++;
				
				if (currNode.getLocation().equals(goal))
					break;
				
				HashMap<MapEdge, GeographicPoint> neighbours = currNode.getNeighbours();
				for (MapEdge edge : neighbours.keySet()) {
					
					if (!visited.contains(edge.getEnd())) {
						
						MapNode addNode = map.get(neighbours.get(edge));
						double tempTime = currNode.getTimeToStart() + ((edge.getDistance())/(edge.getSpeedLimit()));
						
						if (tempTime < addNode.getTimeToStart()) {
							addNode.setTimeToStart(tempTime);
							queue.add(addNode);
							
							List<GeographicPoint> temp = path.get(neighbours.get(edge));
							temp.add(currNode.getLocation());
						}
					}
				}
			}
		}
		System.out.println("Dijkstra: " + count);
		return backTrack(path,goal);
	}
	

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		Set<GeographicPoint> visited = new HashSet<GeographicPoint>();
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>(numVertices, new AStarComparator());
		HashMap<GeographicPoint, List<GeographicPoint>> path = new HashMap<GeographicPoint, List<GeographicPoint>>();
		int count = 0;
		
		if (!map.containsKey(start) || !map.containsKey(goal))
			return null;
		
		for (GeographicPoint temp : map.keySet())
			path.put(temp, new ArrayList<GeographicPoint>());
		
		MapNode startNode = map.get(start);
		startNode.setTimeToStart(0.0);
		startNode.setPredictedTime(0.0);
		startNode.setDistanceToStart(0.0);
		startNode.setPredictedDistance(0.0);
		queue.add(startNode);
		
		while (!queue.isEmpty()) {
			MapNode currNode = queue.poll();
			nodeSearched.accept(currNode.getLocation());
			
			if (!visited.contains(currNode.getLocation())) {
				visited.add(currNode.getLocation());
				count++;
				
				if (currNode.getLocation().equals(goal))
					break;
				
				HashMap<MapEdge, GeographicPoint> neighbours = currNode.getNeighbours();
				for (MapEdge edge : neighbours.keySet()) {
					if (!visited.contains(edge.getEnd())) {
						
						MapNode addNode = map.get(neighbours.get(edge));
						double tempPath = currNode.getDistanceToStart() + edge.getDistance();
						double tempTime = currNode.getTimeToStart() + ((edge.getDistance())/(edge.getSpeedLimit()));

						if (tempTime < addNode.getPredictedTime()) {

							addNode.setDistanceToStart(tempPath);
							addNode.setTimeToStart(tempTime);
							
							double predict = tempPath + edge.getEnd().distance(goal);

							addNode.setPredictedDistance(predict);
							addNode.setPredictedTime(predict/edge.getSpeedLimit());
							
							queue.add(addNode);
							
							List<GeographicPoint> temp = path.get(neighbours.get(edge));
							temp.add(currNode.getLocation());
						}
					}
				}
				
			}
		}
		System.out.println("Astarsearch: " + count);
		return backTrack(path,goal);
	}

	public void printGraph() {
		for (MapNode currNode : map.values()) {
			System.out.println("This is node: " + currNode.getLocation().toString());
			System.out.println("These are the neighbours: ");
			currNode.printAllNeighbours();
		}
	}
	
	public static void main(String[] args)
	{
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);	
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		System.out.println(testroute);
		System.out.println(testroute2);
		
	}

		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		*/
		
	
}
