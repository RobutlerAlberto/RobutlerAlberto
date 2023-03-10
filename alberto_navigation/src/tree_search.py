from abc import ABC, abstractmethod

# Dominios de pesquisa
# Permitem calcular
# as accoes possiveis em cada estado, etc
class SearchDomain(ABC):

    # construtor
    @abstractmethod
    def __init__(self):
        pass

    # lista de accoes possiveis num estado
    @abstractmethod
    def actions(self, state):
        pass

    # resultado de uma accao num estado, ou seja, o estado seguinte
    @abstractmethod
    def result(self, state, action):
        pass

    # custo de uma accao num estado
    @abstractmethod
    def cost(self, state, action):
        pass

    # custo estimado de chegar de um estado a outro
    @abstractmethod
    def heuristic(self, state, goal):
        pass

    # test if the given "goal" is satisfied in "state"
    @abstractmethod
    def satisfies(self, state, goal):
        pass


# Problemas concretos a resolver
# dentro de um determinado dominio
class SearchProblem:
    def __init__(self, domain, initial, goal):
        self.domain = domain
        self.initial = initial
        self.goal = goal
    def goal_test(self, state):
        return self.domain.satisfies(state,self.goal)

# Nos de uma arvore de pesquisa
class SearchNode:
    def __init__(self,state,parent,depth,heuristic): 
        self.state = state
        self.parent = parent
        self.depth = depth
        if self.parent:
            self.cost = self.parent.cost
        else:
            self.cost = 0
        self.heuristic = heuristic
    def __str__(self):
        return "no(" + str(self.state) + "," + str(self.parent) + ")"
    def __repr__(self):
        return str(self)

    def checkForLoops(self,nextState):
        if self.parent == None:
            return False
        else:
            previous = self.parent.checkForLoops(nextState)
            if self.state==nextState:
                return previous or True
            else:
                return previous or False

# Arvores de pesquisa
class SearchTree:

    # construtor
    def __init__(self,problem, strategy='breadth'): 
        self.problem = problem
        root = SearchNode(problem.initial, None,0,self.problem.domain.heuristic(self.problem.initial,self.problem.goal))
        self.open_nodes = [root]
        self.strategy = strategy
        self.solution = None
        self.length = None
        self.terminals = 0
        self.non_terminals = 0
        self.avg_branching = 0
        self.cost = 0

    # obter o caminho (sequencia de estados) da raiz ate um no
    def get_path(self,node):
        if node.parent == None:
            return [node.state]
        path = self.get_path(node.parent)
        path += [node.state]
        return(path)


    # procurar a solucao
    def search(self,limit=None):
        while self.open_nodes != []:
            node = self.open_nodes.pop(0)

            if node.parent:
                C1 = node.parent.state
                C2 = node.state
                action = (C1,C2)
                addedCost = self.problem.domain.cost(C1,action)
                node.cost += addedCost;

            if self.problem.goal_test(node.state):
                self.solution = node
                #self.solution.heuristic = self.problem.domain.heuristic(self.problem.initial,node.state)
                self.cost = node.cost
                self.calclength()
                for n in self.open_nodes:
                    self.terminals += 1
                self.avg_branching = round(((self.terminals + self.non_terminals - 1)/(self.non_terminals)),2)
                return self.get_path(node)
            else:
                self.non_terminals += 1

            lnewnodes = []
            if limit==None or (limit!=None and node.depth <= (limit-1)):
                for a in self.problem.domain.actions(node.state):
                    newstate = self.problem.domain.result(node.state,a)
                    newnode = SearchNode(newstate,node,node.depth+1,self.problem.domain.heuristic(newstate,self.problem.goal))
                    if not node.checkForLoops(newstate):
                        lnewnodes.append(newnode)
                        # newnode.heuristic = self.problem.domain.heuristic(self.problem.initial,newstate)
            self.add_to_open(lnewnodes)

        return None

    def calclength(self):
        self.length = self.solution.depth

    # juntar novos nos a lista de nos abertos de acordo com a estrategia
    def add_to_open(self,lnewnodes):
        if self.strategy == 'breadth':
            self.open_nodes.extend(lnewnodes)
        elif self.strategy == 'depth':
            self.open_nodes[:0] = lnewnodes
        elif self.strategy == 'uniform':
            self.open_nodes.extend(lnewnodes)
            self.open_nodes.sort(key = (lambda node : node.cost))
        elif self.strategy == 'greedy':
            self.open_nodes.extend(lnewnodes)
            self.open_nodes.sort(key = (lambda node : node.heuristic))
        elif self.strategy == 'a*':
            self.open_nodes.extend(lnewnodes)
            self.open_nodes.sort(key = (lambda node : node.cost + node.heuristic))