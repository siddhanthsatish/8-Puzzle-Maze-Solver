import sys
import puzz
import pdqpq

MAX_SEARCH_ITERS = 100000
GOAL_STATE = puzz.EightPuzzleBoard("012345678")


def solve_puzzle(start_state, strategy):
    """Perform a search to find a solution to a puzzle.
    
    Args:
        start_state: an EightPuzzleBoard object indicating the start state for the search
        flavor: a string indicating which type of search to run.  Can be one of the following:
            'bfs' - breadth-first search
            'ucost' - uniform-cost search
            'greedy-h1' - Greedy best-first search using a misplaced tile count heuristic
            'greedy-h2' - Greedy best-first search using a Manhattan distance heuristic
            'greedy-h3' - Greedy best-first search using a weighted Manhattan distance heuristic
            'astar-h1' - A* search using a misplaced tile count heuristic
            'astar-h2' - A* search using a Manhattan distance heuristic
            'astar-h3' - A* search using a weighted Manhattan distance heuristic
    
    Returns: 
        A dictionary containing describing the search performed, containing the following entries:
            'path' - a list of 2-tuples representing the path from the start state to the goal state 
                (both should be included), with each entry being a (str, EightPuzzleBoard) pair 
                indicating the move and resulting state for each action.  Omitted if the search 
                fails.
            'path_cost' - the total cost of the path, taking into account the costs associated 
                with each state transition.  Omitted if the search fails.
            'frontier_count' - the number of unique states added to the search frontier at any
                point during the search.
            'expanded_count' - the number of unique states removed from the frontier and expanded 
                (successors generated).
    """

    if strategy == 'bfs':
        return bfs(start_state)
    if strategy == 'ucost':
        return ucs(start_state)
    if strategy == 'greedy-h1':
        return greedy_h1(start_state)
    if strategy == 'greedy-h2':
        return greedy_h2(start_state)
    if strategy == 'greedy-h3':
        return greedy_h3(start_state)
    if strategy == 'astar-h1':
        return astar_h1(start_state)
    if strategy == 'astar-h2':
        return astar_h2(start_state)
    if strategy == 'astar-h3':
        return astar_h3(start_state)

    results = {
        'path': [],
        'path_cost': 0,
        'frontier_count': 0,
        'expanded_count': 0,
    }

    return results


def back(parent, start, end):
    path = [end]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path


def get_key(dic, val):
    keys = list(dic.keys())
    vals = list(dic.values())
    pos = vals.index(val)
    return keys[pos]


def prettyprint(path):
    for i in path:
        print(i.pretty())
        print()


def path_with_moves(path):
    pm = [("start", path[0])]
    path_cost = 0
    for i in range(len(path)):
        if path[i] != path[-1]:
            pm.append((get_key(path[i].successors(), path[i + 1]), path[i + 1]))
            a, b = path[i + 1].find('0')
            temp = int(path[i]._get_tile(a, b))
            path_cost += temp ** 2
    return pm, path_cost


def bfs(start_state):
    i = 0
    path = []
    parents = dict()
    frontier = pdqpq.PriorityQueue()
    firstElemCheck = pdqpq.PriorityQueue()
    frontier.add(start_state, i)
    firstElemCheck.add(start_state, i)
    f = firstElemCheck.pop()
    explored = set()
    fsize = 1

    if f == GOAL_STATE:
        complete = True
        path = back(parents, start_state, GOAL_STATE)
        pwm, path_cost = path_with_moves(path)
        results = {
            'path': pwm,
            'path_cost': path_cost,
            'frontier_count': fsize,
            'expanded_count': len(explored),
        }
        return results
    else:
        fsize = 1
        explored = set()
        path_cost = 0
        complete = False
        while frontier.empty() == False:
            node = frontier.pop()
            explored.add(node)

            for x in node.successors().values():
                if x not in frontier and x not in explored:
                    parents.update({x: node})
                    if x == GOAL_STATE:
                        complete = True
                        path = back(parents, start_state, GOAL_STATE)
                        pwm, path_cost = path_with_moves(path)
                        results = {
                            'path': pwm,
                            'path_cost': path_cost,
                            'frontier_count': fsize,
                            'expanded_count': len(explored),
                        }
                        return results
                    else:
                        parents.update({x: node})
                        frontier.add(x, i)
                        fsize = fsize + 1
        if not complete:
            results = {
                'frontier_count': fsize,
                'expanded_count': len(explored),
            }
            return results


def ucs(start_state):
    cost = 0
    cumcost = 0
    path = []
    parents = dict()
    explored = set()
    path_cost = 0
    complete = False
    fsize = 1
    frontier_temp = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, cost)
    frontier_temp.add(start_state, cost)
    explored = set()
    while not frontier.empty():
        node = frontier.pop()
        cumcost = frontier_temp.get(node)
        if node == GOAL_STATE:
            complete = True
            path = back(parents, start_state, GOAL_STATE)
            pwm, path_cost = path_with_moves(path)
            results = {
                'path': pwm,
                'path_cost': path_cost,
                'frontier_count': fsize,
                'expanded_count': len(explored),
            }
            return results
        explored.add(node)
        for x in node.successors().values():
            cost = 0
            a, b = x.find('0')
            cost = int(node._get_tile(a, b))
            cost = cost ** 2
            cost += cumcost
            if (x not in frontier and x not in explored) or (x in frontier and frontier.get(x) > cost):
                parents.update({x: node})
                frontier.add(x, cost)
                fsize = fsize + 1
                frontier_temp.add(x, cost)
    if not complete:
        results = {
            'frontier_count': fsize,
            'expanded_count': len(explored),
        }
        return results


# heuristic functions

def h1(x1, x2):
    mis = 0
    for i in range(1, 9):
        a, b = x1.find(str(i))
        p, q = x2.find(str(i))
        if a != p or b != q:
            mis += 1
    return mis


def h2(x1, x2):
    mis = 0
    for i in range(1, 9):
        a, b = x1.find(str(i))
        p, q = x2.find(str(i))
        mis += abs(a - p) + abs(b - q)
    return mis


def h3(x1, x2):
    mis = 0
    for i in range(1, 9):
        a, b = x1.find(str(i))
        p, q = x2.find(str(i))
        mis += (i ** 2) * (abs(a - p) + abs(b - q))
    return mis


# gbfs heuristic 1
def greedy_h1(start_state):
    complete = False
    cost = 0
    cumcost = 0
    path = []
    parents = dict()
    explored = set()
    path_cost = 0
    frontier_temp = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, cost)
    frontier_temp.add(start_state, cost)
    explored = set()
    fsize = 1

    while frontier.empty() == False:
        node = frontier.pop()
        if node == GOAL_STATE:
            complete = True
            path = back(parents, start_state, GOAL_STATE)
            pwm, path_cost = path_with_moves(path)
            # prettyprint(path)
            # print()
            results = {
                'path': pwm,
                'path_cost': path_cost,
                'frontier_count': fsize,
                'expanded_count': len(explored),
            }
            return results
        explored.add(node)
        for x in node.successors().values():
            cost = h1(GOAL_STATE, x)
            if (x not in frontier and x not in explored) or (x in frontier and frontier.get(x) > cost):
                frontier.add(x, cost)
                fsize = fsize + 1
                frontier_temp.add(x, cost)
                parents.update({x: node})
    if not complete:
        results = {
            'frontier_count': fsize,
            'expanded_count': len(explored),
        }
        return results


# gbfs heuristic 2
def greedy_h2(start_state):
    cost = 0
    fsize = 1
    cumcost = 0
    path = []
    parents = dict()
    explored = set()
    path_cost = 0
    complete = False
    frontier_temp = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, cost)
    frontier_temp.add(start_state, cost)
    explored = set()

    while frontier.empty() == False:
        node = frontier.pop()
        if node == GOAL_STATE:
            complete = True
            path = back(parents, start_state, GOAL_STATE)
            pwm, path_cost = path_with_moves(path)
            results = {
                'path': pwm,
                'path_cost': path_cost,
                'frontier_count': fsize,
                'expanded_count': len(explored),
            }
            return results
        explored.add(node)
        for x in node.successors().values():
            cost = h2(GOAL_STATE, x)
            if (x not in frontier and x not in explored) or (x in frontier and frontier.get(x) > cost):
                frontier.add(x, cost)
                fsize = fsize + 1
                frontier_temp.add(x, cost)
                parents.update({x: node})
    if not complete:
        results = {
            'frontier_count': fsize,
            'expanded_count': len(explored),
        }
        return results


# gbfs heuristic 3
def greedy_h3(start_state):
    cost = 0
    cumcost = 0
    complete = False
    path = []
    parents = dict()
    explored = set()
    path_cost = 0
    frontier_temp = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, cost)
    frontier_temp.add(start_state, cost)
    explored = set()
    fsize = 1
    while frontier.empty() == False:
        node = frontier.pop()
        if node == GOAL_STATE:
            complete = True
            path = back(parents, start_state, GOAL_STATE)
            pwm, path_cost = path_with_moves(path)
            results = {
                'path': pwm,
                'path_cost': path_cost,
                'frontier_count': fsize,
                'expanded_count': len(explored),
            }
            return results
        explored.add(node)
        for x in node.successors().values():
            cost = h3(GOAL_STATE, x)
            if (x not in frontier and x not in explored) or (x in frontier and frontier.get(x) > cost):
                frontier.add(x, cost)
                fsize = fsize + 1
                frontier_temp.add(x, cost)
                parents.update({x: node})
    if not complete:
        results = {
            'frontier_count': fsize,
            'expanded_count': len(explored),
        }
        return results


# a* h1
def astar_h1(start_state):
    cost = 0
    cumcost = 0
    path = []
    parents = dict()
    explored = set()
    path_cost = 0
    frontier_temp = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, cost)
    frontier_temp.add(start_state, cost)
    explored = set()
    complete = False
    fsize = 1

    while frontier.empty() == False:
        node = frontier.pop()
        cumcost = frontier_temp.get(node)
        cumcost -= h1(GOAL_STATE, node)
        if node == GOAL_STATE:
            complete = True
            path = back(parents, start_state, GOAL_STATE)
            pwm, path_cost = path_with_moves(path)
            results = {
                'path': pwm,
                'path_cost': path_cost,
                'frontier_count': fsize,
                'expanded_count': len(explored),
            }
            return results
        explored.add(node)
        p, q = node.find('0')
        for x in node.successors().values():
            cost = 0
            a, b = x.find('0')
            cost = cumcost + (int(node._get_tile(a, b))) ** 2 + h1(GOAL_STATE, x)
            # cost = cumcost + (int(node._get_tile(a, b))) ** 2 + h1(node, x)
            # # print(cost)
            # cumcost += (int(node._get_tile(a, b))) ** 2
            # print(cumcost)
            if (x not in frontier and x not in explored) or (x in frontier and frontier.get(x) > cost):
                frontier.add(x, cost)
                fsize += 1
                # frontier_temp.add(x, cumcost)
                frontier_temp.add(x, cost)
                parents.update({x: node})
    if not complete:
        results = {
            'frontier_count': fsize,
            'expanded_count': len(explored),
        }
        return results


# astar heuristic 2
def astar_h2(start_state):
    cost = 0
    fsize = 1
    cumcost = 0
    complete = False
    path = []
    parents = dict()
    explored = set()
    path_cost = 0
    frontier_temp = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, cost)
    frontier_temp.add(start_state, cost)
    explored = set()

    while frontier.empty() == False:
        node = frontier.pop()
        cumcost = frontier_temp.get(node) - h2(GOAL_STATE, node)
        if node == GOAL_STATE:
            complete = True
            path = back(parents, start_state, GOAL_STATE)
            pwm, path_cost = path_with_moves(path)

            results = {
                'path': pwm,
                'path_cost': path_cost,
                'frontier_count': fsize,
                'expanded_count': len(explored),
            }
            return results
        explored.add(node)
        p, q = node.find('0')
        for x in node.successors().values():
            cost = 0
            a, b = x.find('0')
            cost = cumcost + (int(node._get_tile(a, b))) ** 2 + h2(GOAL_STATE, x)
            # cost = cumcost + (int(node._get_tile(a, b))) ** 2 + h2(node, x)
            # cumcost += (int(node._get_tile(a, b))) ** 2
            if (x not in frontier and x not in explored) or (x in frontier and frontier.get(x) > cost):
                frontier.add(x, cost)
                frontier_temp.add(x, cost)
                # frontier_temp.add(x, cost)
                fsize += 1
                parents.update({x: node})
    if not complete:
        results = {
            'frontier_count': fsize,
            'expanded_count': len(explored),
        }
        return results


# astar heuristic 3
def astar_h3(start_state):
    complete = False
    cost = 0
    cumcost = 0
    path = []
    parents = dict()
    explored = set()
    path_cost = 0
    frontier_temp = pdqpq.PriorityQueue()
    frontier = pdqpq.PriorityQueue()
    frontier.add(start_state, cost)
    frontier_temp.add(start_state, cost)
    explored = set()
    fsize = 1

    while frontier.empty() == False:
        node = frontier.pop()
        cumcost = frontier_temp.get(node) - h3(GOAL_STATE, node)
        if node == GOAL_STATE:
            complete = True
            path = back(parents, start_state, GOAL_STATE)
            pwm, path_cost = path_with_moves(path)
            # prettyprint(path)
            results = {
                'path': pwm,
                'path_cost': path_cost,
                'frontier_count': fsize,
                'expanded_count': len(explored),
            }
            return results
        explored.add(node)
        p, q = node.find('0')
        for x in node.successors().values():
            cost = 0
            a, b = x.find('0')
            cost = cumcost + (int(node._get_tile(a, b))) ** 2 + h3(GOAL_STATE, x)
            # cost = cumcost + (int(node._get_tile(a, b))) ** 2 + h3(node, x)
            # cumcost += (int(node._get_tile(a, b))) ** 2
            if (x not in frontier and x not in explored) or (x in frontier and frontier.get(x) > cost):
                frontier.add(x, cost)
                frontier_temp.add(x, cost)
                # frontier_temp.add(x, cost)
                fsize += 1
                parents.update({x: node})
    if not complete:
        results = {
            'frontier_count': fsize,
            'expanded_count': len(explored),
        }
        return results


def print_summary(results):
    if 'path' in results:
        print("found solution of length {}, cost {}".format(len(results['path']),
                                                            results['path_cost']))
        for move, state in results['path']:
            print("  {:5} {}".format(move, state))
    else:
        print("no solution found")
    print("{} states placed on frontier, {} states expanded".format(results['frontier_count'],
                                                                    results['expanded_count']))


############################################

if __name__ == '__main__':
    start = puzz.EightPuzzleBoard(sys.argv[1])
    method = sys.argv[2]
    # start = puzz.EightPuzzleBoard('802356174')
    # method= 'ucs'
    print("solving puzzle {} -> {}".format(start, GOAL_STATE))
    # greedy_h1(start)
    results = solve_puzzle(start, method)
    print(results)
    print_summary(results)
