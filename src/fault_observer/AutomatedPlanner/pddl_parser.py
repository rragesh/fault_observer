import sys
import pprint
import re

from itertools import product
import operator as ops

NUM_OPS = {
    '>' : ops.gt,
    '<' : ops.lt,
    '=' : ops.eq,
    '>=': ops.ge,
    '<=': ops.le
}
#negative effects
def neg(effect):
    """
    Makes the given effect a negative (delete) effect, like 'not' in PDDL.
    """
    return (-1, effect) #(-1,(a,b))
def _grounder(arg_names, args):
    """
    Returns a function for grounding predicates and function symbols
    """
    namemap = dict()
    for arg_name, arg in zip(arg_names, args):
        namemap[arg_name] = arg
    def _ground_by_names(predicate):
        predicate = tuple(predicate)
        return predicate[0:1] + tuple(namemap.get(arg, arg) for arg in predicate[1:])
    return _ground_by_names

def _num_pred(op, x, y):
    """
    Returns a numerical predicate that is called on a State.
    """
    def predicate(state):
        operands = [0, 0]
        for i, o in enumerate((x, y)):
            if type(o) == int:
                operands[i] = o
            else:
                operands[i] = state.f_dict[o]
        return op(*operands)
    return predicate

class Domain(object):

    def __init__(self, actions=(),name=""):
        self.actions = tuple(actions)

    def ground(self, objects):
        grounded_actions = list()
        for action in self.actions:
            param_lists = [objects[t] for t in action.types]
            param_combos = set() # { (1,a),(2,b) }
            for params in product(*param_lists):
                param_set = frozenset(params)
                if action.unique and len(param_set) != len(params):
                    continue
                if action.no_permute and param_set in param_combos:
                    continue
                param_combos.add(param_set)
                grounded_actions.append(action.ground(*params))
        return grounded_actions

class Action(object):
    def __init__(self, name, parameters=(), preconditions=(),add_effects=(),del_effects=(),
                 unique=True, no_permute=False):
        self.name = name
        #if parameters:
        if len(parameters) > 0:
            # assigning Object types and parameters
            self.types = ['objects']*len(parameters) #['object','object']....
            self.arg_names = tuple(parameters) #[?x,?y,?z]....
        else:
            self.types = tuple()
            self.arg_names = tuple()
        for i in del_effects:
            add_effects.append(neg(i))

        self.preconditions = preconditions
        self.effects = add_effects
        self.unique = unique
        self.no_permute = no_permute

    def ground(self, *args):
        return _GroundedAction(self, *args)

    def __str__(self):
        arglist = ', '.join(['%s - %s' % pair for pair in zip(self.arg_names, self.types)])
        return '%s(%s)' % (self.name, arglist)

class _GroundedAction(object):
    def __init__(self, action, *args):
        self.name = action.name
        ground = _grounder(action.arg_names, args)
        # Ground Action Signature
        self.sig = ground((self.name,) + action.arg_names)
        # Ground Preconditions
        self.preconditions = list()
        self.num_preconditions = list()
        for pre in action.preconditions:
            if pre[0] in NUM_OPS:
                operands = [0, 0]
                for i in range(2):
                    if type(pre[i + 1]) == int:
                        operands[i] = pre[i + 1]
                    else:
                        operands[i] = ground(pre[i + 1])
                np = _num_pred(NUM_OPS[pre[0]], *operands)
                self.num_preconditions.append(np)
            else:
                self.preconditions.append(ground(pre))

        # Ground Effects
        self.add_effects = list()
        self.del_effects = list()
        self.num_effects = list()
        for effect in action.effects:
            if effect[0] == -1:
                self.del_effects.append(ground(effect[1]))
            elif effect[0] == '+=':
                function = ground(effect[1])
                value = effect[2]
                self.num_effects.append((function, value))
            elif effect[0] == '-=':
                function = ground(effect[1])
                value = -effect[2]
                self.num_effects.append((function, value))
            else:
                self.add_effects.append(ground(effect))

        # print("Action:%s\nAdd_effects:%s\nDel_effects:%s\n\n"%(self.sig,self.add_effects,self.del_effects))

        self.act_result = {self.sig:{"precondtions":self.preconditions,"add_effects":self.add_effects,"del_effects":self.del_effects}}

    def __str__(self):
        arglist = ', '.join(map(str, self.sig[1:]))
        return '%s(%s)' % (self.sig[0], arglist)

class State(object):

    def __init__(self, predicates, functions, cost=0, predecessor=None):
        """Represents a state for A* search"""
        predicates = [tuple(i) for i in predicates]
        self.predicates = frozenset(predicates)
        self.functions = tuple(functions.items())
        self.f_dict = functions
        self.predecessor = predecessor
        self.cost = cost

    def is_true(self, predicates):
        predicates = [tuple(i) for i in predicates]
        if set(predicates).issubset(set(self.predicates)) == True:
            return True
        else:
            return False

    def apply(self, action, monotone=False):
        """
        Apply the action to this state to produce a new state.
        If monotone, ignore the delete list (for A* heuristic)
        """
        new_preds = set(self.predicates)
        new_preds |= set(action.add_effects)
        if not monotone:
            new_preds -= set(action.del_effects)
        new_functions = dict()
        new_functions.update(self.functions)
        for function, value in action.num_effects:
            new_functions[function] += value
        return State(new_preds, new_functions, self.cost + 1, (self, action))

    def plan(self):
        """
        Follow backpointers to successor states
        to produce a plan.
        """
        plan = list()
        n = self
        while n.predecessor is not None:
            plan.append(n.predecessor[1])
            n = n.predecessor[0]
        plan.reverse()
        return plan

    # Implement __hash__ and __eq__ so we can easily
    # check if we've encountered this state before

    def __hash__(self):
        return hash((self.predicates, self.functions))

    def __eq__(self, other):
        return ((self.predicates, self.functions) ==
                (other.predicates, other.functions))

    def __str__(self):
        return ('Predicates:\n%s' % '\n'.join(map(str, self.predicates))
                +'\nFunctions:\n%s' % '\n'.join(map(str, self.functions)))
    def __lt__(self, other):
        return hash(self) < hash(other)


class Problem(object):

    def __init__(self, domain, objects, init=(), goal=()):
        # Ground actions from domain
        self.grounded_actions = domain.ground(objects)

        # Parse Initial State
        predicates = list()
        functions = dict()

        for predicate in init:
            if predicate[0] == '=':
                functions[predicate[1]] = predicate[2]
            else:
                predicates.append(predicate)
        self.initial_state = State(predicates, functions)


        # Parse Goal State
        self.goals = list()
        self.num_goals = list()
        for g in goal:
            if g[0] in NUM_OPS:
                ng = _num_pred(NUM_OPS[g[0]], *g[1:])
                self.num_goals.append(ng)
            else:
                self.goals.append(g)

class PDDL_Parser():

    def __init__(self,domain_file = None,problem_file = None):
        if domain_file == None or problem_file == None:
            raise ValueError("Please input domain/problem pddl file")
        self.parse_domain(domain_file) #parse domain file
        self.parse_problem(problem_file) #parse problem file

    #scan the tokens and its values from file
    def scan_tokens(self, filename):
        '''
        Scan the entire PDDL file and store in the form of list
        '''
        # open PDDL file and convert into lower case
        with open(filename,'r') as f:
            str = re.sub(r';.*$', '', f.read(), flags=re.MULTILINE).lower()
        # Tokenize
        stack = []
        list = []
        # iterate throught the entire PDDL file
        for t in re.findall(r'[()]|[^\s()]+', str):
            # append temp list from open parantheses
            if t == '(':
                stack.append(list)
                list = []
            # pop temp list and append to main list
            elif t == ')':
                if stack:
                    l = list
                    list = stack.pop()
                    list.append(l)
                else:
                    raise Exception('Missing open parentheses')
            else:
                list.append(t)
        if stack:
            raise Exception('Missing close parentheses')
        if len(list) != 1:
            raise Exception('Malformed expression')
        return list[0]

    def parse_domain(self, domain_file):
        tokens = self.scan_tokens(domain_file)
        if type(tokens) is list and tokens.pop(0) == 'define':
            self.domain_name = 'unknown'
            self.actions = []
            while tokens:
                group = tokens.pop(0)
                t = group.pop(0)
                if   t == 'domain':
                    self.domain_name = group[0]
                elif t == ':requirements':
                    pass # TODO
                elif t == ':predicates':
                    pass # TODO
                elif t == ':action':
                    # if token finds 'action' variable,parse action
                    self.parse_action(group)
                else: print(str(t) + ' is not recognized in domain')
            self.domain = Domain(actions=self.actions,name=self.domain_name)
        else:
            raise 'File ' + domain_filename + ' does not match domain pattern'

    def parse_action(self,group):
        name = group.pop(0)
        if not type(name) is str:
            raise Exception('Action without name definition')
        for act in self.actions:
            if act.name == name:
                raise Exception('Action ' + name + ' redefined')
        parameters = []
        positive_preconditions = []
        negative_preconditions = []
        add_effects = []
        del_effects = []
        while group:
            t = group.pop(0)
            if t == ':parameters':
                if not type(group) is list:
                    raise Exception('Error with ' + name + ' parameters')
                parameters = group.pop(0)
            elif t == ':precondition':
                self.split_propositions(group.pop(0), positive_preconditions, negative_preconditions, name, ' preconditions')
            elif t == ':effect':
                self.split_propositions(group.pop(0), add_effects, del_effects, name, ' effects')
            else: print(str(t) + ' is not recognized in action')
        self.actions.append(Action(name, parameters, positive_preconditions, add_effects, del_effects))

#Split the token values as list
    def split_propositions(self, group, pos, neg, name, part):
        if not type(group) is list:
            raise Exception('Error with ' + name + part)
        if group[0] == 'and':
            group.pop(0)
        else:
            group = [group]
        for proposition in group:
            if proposition[0] == 'not':
                if len(proposition) != 2:
                    raise Exception('Unexpected not in ' + name + part)
                neg.append(proposition[-1])
            else:
                pos.append(proposition)

    def parse_problem(self, problem_file):
        tokens = self.scan_tokens(problem_file)
        if type(tokens) is list and tokens.pop(0) == 'define':
            self.problem_name = 'unknown'
            self.objects = []
            self.state = []
            self.positive_goals = []
            self.negative_goals = []
            while tokens:
                group = tokens.pop(0)
                t = group[0]
                if   t == 'problem':
                    self.problem_name = group[-1]
                elif t == ':domain':
                    if self.domain_name != group[-1]:
                        raise Exception('Different domain specified in problem file')
                # elif t == ':requirements':
                #     pass # TODO
                elif t == ':objects':
                    group.pop(0)
                    self.objects = group
                elif t == ':init':
                    group.pop(0)
                    self.state = group
                elif t == ':goal':
                    self.split_propositions(group[1], self.positive_goals, self.negative_goals, '', 'goals')
                else: print(str(t) + ' is not recognized in problem')
            #Initialize problem as an object
            self.problem = Problem(self.domain,{'objects':self.objects},init=self.state, goal= self.positive_goals+self.negative_goals)
