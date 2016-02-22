import sys,re
import math
import time
import difflib
from JSGFParser import JSGFParser

debug = True

class nlp():

    def __init__(self):
        self.grammar = JSGFParser('speech/hark-sphinx/grammar/NielsSebastiaan.gram')
        self.names = self.grammar.findVariableItems("<start>", includeVars=False)
        self.locations = self.grammar.findVariableItems("<location>", includeVars=False)
        self.verbs = self.grammar.findVariableItems("<verb>", includeVars=False)
        self.nav = self.grammar.findVariableItems("<nav>", includeVars=False)

    def remove_name(self,s):
        for name in self.names:
            start = "%s " % name
            if  s.startswith(start):
                s = s.lstrip(start)
        return s


    def remove_opts(self, hypstr):
        return self.grammar.filterOptionals(hypstr)

    def find_type(self,string):
        if self.remove_opts(string) in self.locations: return "location"
        if self.remove_opts(string) in self.verbs: return "verb"
        return None

    def split_order(self, s1,s2):
        splits = []
        regex = re.compile(" and ")
        matches1 = re.split(regex, s1)
        matches2 = re.split(regex, s2)
        for x, y in zip(matches1,matches2):
            splits.append([x,y])
        return splits

    def partial_parse(self,order):
        parsed_list = []
        for s in order:
            order_verb = None
            order_location = None
            order_item = None
            regex = re.compile("(%s)" % '|'.join(self.locations))
            match = re.split(regex, self.remove_name(s))

            if len(match)<3: return None
            order_location = match[1]
                       
            regex = re.compile("(%s)" % '|'.join(self.verbs))
            order_verb_grp = re.search(regex,match[0])
            if order_verb_grp != None: order_verb = order_verb_grp.group(0)    
            regex = re.compile("(%s)" % '|'.join(self.items))
            
            order_reg = re.search(regex,match[0])
            if order_reg != None: 
                order_item = order_reg.group(0)
            else:
                order_item = "it"

            parsed_list.append({"verb" : order_verb, "location" : order_location})
        return parsed_list

    def needs_question(self,parsed_order):
        o1 = parsed_order[0]
        o2 = parsed_order[1]
        location = None
        verb = None

        # we have a perfect match, return to sender
        if o2 == o1: return {"diff" : False , "question" : "No difference found", "understood_options": None, "action" : o1,'unknown': None}

        if o1['location'] == o2['location'] : location = o1['location']
        if o1['verb'] == o2['verb'] : verb = o1['verb']

        # generate questions for the misheard variables TODO not complete        

        # @TODO you should make use of the information here to return a proper action structure








        # else
        return {"diff" : True , "question" : "could you repeat this part?", "understood_options": None, "action" : None,'unknown': None}

    def confirm(self, action):
        v = action['action']['verb']
        l = action['action']['location']
        confirmation = ""

        if v == "approach":
            confirmation = "I will approach the " + l
        else:
            #v in self.nav
            confirmation = "I will go to the " + l

        return confirmation

    def get_actions(self, speech_observation):
        actions = []
        # we first split the order with they keyword "and"
        orders = self.split_order(self.remove_opts(speech_observation['2best'][0]),self.remove_opts(speech_observation['2best'][1]))
        for order in orders:
            parsed_order = self.partial_parse(order)
            if parsed_order != None:
                actions.append(self.needs_question(parsed_order))
            else:
                print "could not parse. not in grammar"
        return actions


if __name__ == '__main__':
    actionqueue = []
    nlp = nlp()
    #result = comm.analyze2best('alice get cup from table one and empty it in bowl on table two',        'alice get cup from table one and empty it in bowl on table one')
    #print comm.resultResponse(result)
    #print comm.parseShortSentence('alice get cup from table one and empty basket on table two')
    testcase = []
    testcase.append({'2best' : ["alice get cup from table one and empty bowl table two", "alice get cup from table two and empty basket table two"]})
    testcase.append({'2best' : ["alice get cup from table one and bring it to table three", "alice get cup from table two and bring to table three"]})
    testcase.append({'2best': ["alice please get me a cup from table one","alice please get me a cup from table one"]})

    for obs in testcase:
        print "I heard one of these:"
        print obs['2best'][0]
        print obs['2best'][1]
        print ''
        actions = nlp.get_actions(obs)

        first_action = True
        for item in actions:
            
            if item['diff']: #we have something to do before adding action to queue
                print "[Alice says]",
                if not first_action: print "... but",
                print item['question']
                answer = raw_input('> ')
                item['action'][item['unknown']] = answer

            actionqueue.append(item['action'])
            print "[Alice says]",
            if first_action : 
                print "Okay,",    
            else:
                print "then",
            print nlp.confirm(item)
            first_action = False

        print "\nactionqueue:"
        last_object = None
        for item in actionqueue:
            print item

