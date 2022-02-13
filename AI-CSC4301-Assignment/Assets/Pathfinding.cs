using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;


public class Pathfinding : MonoBehaviour
{
	public Transform seeker, target;
	Grid grid;
	int countTopology1 = 0;
	int countTopology2 = 0;
	int countUCS = 0;
	int countBFS = 0;
	int countDFS = 0;

	void Awake()
	{
		grid = GetComponent<Grid>();
	}

	void Update()
	{
		countTopology1 = 0;
		countTopology2 = 0;
		countUCS = 0;
		countBFS = 0;
		countDFS = 0;

		var watchTopology1 = new System.Diagnostics.Stopwatch();
		var watchTopology2 = new System.Diagnostics.Stopwatch();
		var watchUCS = new System.Diagnostics.Stopwatch();
		var watchBFS = new System.Diagnostics.Stopwatch();
		var watchDFS = new System.Diagnostics.Stopwatch();

		watchTopology1.Start();
		FindPathTopology1(seeker.position, target.position);
		watchTopology1.Stop();

		watchTopology2.Start();
		FindPathTopology2(seeker.position, target.position);
		watchTopology2.Stop();

		watchUCS.Start();
		FindPathUCS(seeker.position, target.position);
		watchUCS.Stop();

		watchBFS.Start();
		FindPathBFS(seeker.position, target.position);
		watchBFS.Stop();

		watchDFS.Start();
		FindPathDFS(seeker.position, target.position);
		watchDFS.Stop();

		Debug.Log($"Execution of Time A* Topological Distance Dim 2: {watchTopology2.ElapsedMilliseconds} ms, retracement : {countTopology2}\nExecution Time A* Topological Distance Dim 1: {watchTopology1.ElapsedMilliseconds} ms, retracement : {countTopology1}\nExecution Time UCS: {watchUCS.ElapsedMilliseconds} ms, retracement : {countUCS}\nExecution Time BFS: {watchBFS.ElapsedMilliseconds} ms, retracement : {countBFS}\nExecution Time DFS: {watchDFS.ElapsedMilliseconds} ms, retracement : {countDFS}");
	}


	void FindPathTopology1(Vector3 startPos, Vector3 targetPos)
	{
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				RetracePathTopology1(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceTopology1(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistanceTopology1(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void FindPathTopology2(Vector3 startPos, Vector3 targetPos)
	{
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				RetracePathTopology2(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistanceTopology2(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistanceTopology2(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void FindPathDFS(Vector3 startPos, Vector3 targetPos)
	{
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
		Stack<Node> StackDFS = new Stack<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		StackDFS.Push(startNode);

		while (StackDFS.Count != 0)
		{
			Node currentNode = StackDFS.Pop();
			if (currentNode == targetNode)
			{
				RetracePathDFS(startNode, targetNode);
				return;
			}
			closedSet.Add(currentNode);
			foreach (Node neighbour in grid.GetNeighbours(currentNode))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}
				if (neighbour.walkable || !StackDFS.Contains(neighbour))
				{
					closedSet.Add(neighbour);
					neighbour.parent = currentNode;
					StackDFS.Push(neighbour);
				}
			}
		}
	}


	void FindPathBFS(Vector3 startPos, Vector3 targetPos)
	{

		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);
		Queue<Node> queueBFS = new Queue<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		queueBFS.Enqueue(startNode);

		while (queueBFS.Count != 0)
		{
			Node currentNode = queueBFS.Dequeue();
			if (currentNode == targetNode)
			{
				RetracePathBFS(startNode, targetNode);
				return;
			}
			closedSet.Add(currentNode);
			foreach (Node neighbour in grid.GetNeighbours(currentNode))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}
				if (neighbour.walkable || !queueBFS.Contains(neighbour))
				{
					closedSet.Add(neighbour);
					neighbour.parent = currentNode;
					queueBFS.Enqueue(neighbour);
				}
			}
		}
	}


	void FindPathUCS(Vector3 startPos, Vector3 targetPos)
	{

		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0)
		{
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++)
			{
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode)
			{
				RetracePathUCS(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node))
			{
				if (!neighbour.walkable || closedSet.Contains(neighbour))
				{
					continue;
				}

				int newCostToNeighbour = node.gCost;
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
				{
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = 0;
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}


	void RetracePathTopology1(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			countTopology1++;
		}
		path.Reverse();
		grid.pathTopology1 = path;
	}

	void RetracePathTopology2(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			countTopology2++;
		}
		path.Reverse();
		grid.pathTopology2 = path;
	}


	void RetracePathUCS(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			countUCS++;
		}
		path.Reverse();
		grid.pathUCS = path;
	}


	void RetracePathBFS(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			countBFS++;
		}
		path.Reverse();
		grid.pathBFS = path;
	}


	void RetracePathDFS(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			countDFS++;
		}
		path.Reverse();
		grid.pathDFS = path;
	}


	int GetDistanceTopology1(Node nodeA, Node nodeB)
	{
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
		return dstX + dstY;
	}

	int GetDistanceTopology2(Node nodeA, Node nodeB)
	{
		return (int)Mathf.Sqrt((Mathf.Pow(nodeA.gridX - nodeB.gridX, 2) + Mathf.Pow(nodeA.gridY - nodeB.gridY, 2)));
	}
}