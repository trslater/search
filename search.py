from __future__ import annotations
from heapq import heappop, heappush
from typing import NamedTuple


class Problem(NamedTuple):
    """Search Problem Definition
    
    Only knows about state, not about nodes.
    """

    init_state: str
    goal_state: str
    states: set[str]
    successors: dict[str, str]

    def step_cost(self, from_state, to_state):
        return 1


class Node(NamedTuple):
    state: str
    parent: Node = None
    path_cost: int = 0

    @classmethod
    def from_parent(cls, problem, parent, state):
        path_cost = parent.path_cost + problem.step_cost(parent.state, state)

        return cls(state, parent, path_cost)

    def __lt__(self, other):
        return self.path_cost < other.path_cost


def main():
    problem = Problem(
        "Vancouver", "Peachland",
        {"Vancouver", "Peachland", "Merritt", "Hope"},
        {
            "Vancouver": {"Hope"},
            "Hope": {"Vancouver", "Merritt"},
            "Merritt": {"Hope", "Peachland"},
            "Peachland": {"Merritt"}})

    solution = search(problem)
    
    step = solution
    
    while True:
        print(step.state)
        step = step.parent

        if not step:
            break


def search(problem):
    frontier = []
    root_node = Node(problem.init_state)
    heappush(frontier, root_node)
    explored = set()

    while True:
        if not frontier:
            raise Exception("No solution")
        
        node = heappop(frontier)

        if node.state == problem.goal_state:
            return node

        if node.state not in explored:
            explored.add(node.state)
            for successor in problem.successors[node.state]:
                heappush(frontier, Node.from_parent(problem, node, successor))


if __name__ == "__main__":
    main()
