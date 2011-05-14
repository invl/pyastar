
def astar(start, goal, successors, cost_between, evaluate):
    """
    A* Path Finding Algorithm

    Args:
        start:          start node
        goal:           goal node
        graph:          graph[node] = set(successors of node)
        successors:     successors(node) = successors of node
        cost_between:   cost_between(u, v) = cost between node u and v
        evaluate:       evaluate(v) = evaluative cost of node v
    Returns:
        list of nodes in the path or None if failed
    """
    openset = set()
    closeset = set()

    cost = {}
    evaluation = {}
    priority = {}
    came_from = {}

    openset.add(start)
    cost[start] = 0
    evaluation[start] = evaluate(start)
    priority[start] = -cost[start]-evaluation[start]

    def reconstruct_path(v):
        if v == start:
            return [v]
        else:
            return reconstruct_path(came_from[v]) + [v]

    def reconstruct_path2(v):
        """No recursion version"""
        path = []
        while True:
            path.append(v)
            if v == start:
                break
            v = came_from[v]
        return path[::-1]

    while openset:
        node = max(openset, key=lambda k: priority[k])

        if node == goal:
            return reconstruct_path(node)

        openset.discard(node)
        closeset.add(node)

        for succ in successors(node):
            if succ in closeset:
                continue

            cur_cost = cost[node] + cost_between(node, succ)

            if succ not in openset:
                openset.add(succ)
                better = True
            elif cur_cost < cost[succ]:
                better = True
            else:
                better = False

            if better:
                cost[succ] = cur_cost
                evaluation[succ] = evaluate(succ)
                priority[succ] = -cost[succ]-evaluation[succ]
                came_from[succ] = node
