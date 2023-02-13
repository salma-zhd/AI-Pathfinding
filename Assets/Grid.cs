using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Grid : MonoBehaviour
{

	public bool onlyDisplayPathGizmos;
	public LayerMask unwalkableMask;
	public Vector2 gridWorldSize;
	public float nodeRadius;
	Node[,] grid;

	float nodeDiameter;
	int gridSizeX, gridSizeY;

	void Start()
	{
		nodeDiameter = nodeRadius * 2;
		gridSizeX = Mathf.RoundToInt(gridWorldSize.x / nodeDiameter);
		gridSizeY = Mathf.RoundToInt(gridWorldSize.y / nodeDiameter);
		CreateGrid();
	}

	public int MaxSize
	{
		get
		{
			return gridSizeX * gridSizeY;
		}
	}

	void CreateGrid()
	{
		grid = new Node[gridSizeX, gridSizeY];
		Vector3 worldBottomLeft = transform.position - Vector3.right * gridWorldSize.x / 2 - Vector3.forward * gridWorldSize.y / 2;

		for (int x = 0; x < gridSizeX; x++)
		{
			for (int y = 0; y < gridSizeY; y++)
			{
				Vector3 worldPoint = worldBottomLeft + Vector3.right * (x * nodeDiameter + nodeRadius) + Vector3.forward * (y * nodeDiameter + nodeRadius);
				bool walkable = !(Physics.CheckSphere(worldPoint, nodeRadius, unwalkableMask));
				grid[x, y] = new Node(walkable, worldPoint, x, y);
			}
		}
	}

	public List<Node> GetNeighbours(Node node)
	{
		List<Node> neighbours = new List<Node>();

		for (int x = -1; x <= 1; x++)
		{
			for (int y = -1; y <= 1; y++)
			{
				if (x == 0 && y == 0)
					continue;

				int checkX = node.gridX + x;
				int checkY = node.gridY + y;

				if (checkX >= 0 && checkX < gridSizeX && checkY >= 0 && checkY < gridSizeY)
				{
					neighbours.Add(grid[checkX, checkY]);
				}
			}
		}

		return neighbours;
	}


	public Node NodeFromWorldPoint(Vector3 worldPosition)
	{
		float percentX = (worldPosition.x + gridWorldSize.x / 2) / gridWorldSize.x;
		float percentY = (worldPosition.z + gridWorldSize.y / 2) / gridWorldSize.y;
		percentX = Mathf.Clamp01(percentX);
		percentY = Mathf.Clamp01(percentY);

		int x = Mathf.RoundToInt((gridSizeX - 1) * percentX);
		int y = Mathf.RoundToInt((gridSizeY - 1) * percentY);
		return grid[x, y];
	}

	public List<Node> BFSpath;
	public List<Node> DFSpath;
	public List<Node> UCSpath;
	public List<Node> AStarpath1;
	public List<Node> AStarpath2;

	void OnDrawGizmos()
	{
		Gizmos.DrawWireCube(transform.position, new Vector3(gridWorldSize.x, 1, gridWorldSize.y));

		if (onlyDisplayPathGizmos)
		{
			if (BFSpath != null)
			{
				foreach (Node n in BFSpath)
				{
					Gizmos.color = Color.magenta;
					Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - .1f));
				}
			}

			if (DFSpath != null)
			{
				foreach (Node n in DFSpath)
				{
					Gizmos.color = Color.green;
					Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - .1f));
				}
			}

			if (UCSpath != null)
			{
				foreach (Node n in UCSpath)
				{
					Gizmos.color = Color.yellow;
					Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - .1f));
				}
			}

			if (AStarpath1 != null)
			{
				foreach (Node n in AStarpath1)
				{
					Gizmos.color = Color.blue;
					Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - .1f));
				}
			}

			if (AStarpath2 != null)
			{
				foreach (Node n in AStarpath2)
				{
					Gizmos.color = Color.cyan;
					Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - .1f));
				}
			}
		}
		else
		{
			if (grid != null)
			{
				foreach (Node n in grid)
				{
					Gizmos.color = (n.walkable) ? Color.white : Color.red;
					if (BFSpath != null)
						if (BFSpath.Contains(n))
							Gizmos.color = Color.magenta;

					if (DFSpath != null)
						if (DFSpath.Contains(n))
							Gizmos.color = Color.green;

					if (UCSpath != null)
						if (UCSpath.Contains(n))
							Gizmos.color = Color.yellow;

					if (AStarpath1 != null)
						if (AStarpath1.Contains(n))
							Gizmos.color = Color.blue;

					if (AStarpath2 != null)
						if (AStarpath2.Contains(n))
							Gizmos.color = Color.cyan;

					Gizmos.DrawCube(n.worldPosition, Vector3.one * (nodeDiameter - .1f));
				}
			}
		}
	}
}