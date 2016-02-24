import sys,re
import math
import time
import difflib
from JSGFParser import JSGFParser

debug = True

class nlp4():

    def __init__(self):
        self.grammar = JSGFParser('speech/hark-sphinx/grammar/NielsSebastiaan.gram')
        self.names = self.grammar.findVariableItems("<start>", includeVars=False)
        self.locations = self.grammar.findVariableItems("<location>", includeVars=False)
        self.verbs = self.grammar.findVariableItems("<verb>", includeVars=False)
        self.questions = self.grammar.findVariableItems("<sentence>", includeVars=False)

    def remove_name(self,s):
        for name in self.names:
            s = s.lower()
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
            order_question = None
            regex = re.compile("(%s)" % '|'.join(self.locations))
            match = re.split(regex, self.remove_name(s))
            noname = self.remove_name(s)

            if noname == s:
                if len(match)<3:
                #Not a room navigation
                    regex = re.compile("(approach the dining table)")
                    approach_grp = re.search(regex,noname)
                    if approach_grp != None:
                        parsed_list.append({"verb" : "approach", "location" : "dining table", "question" : None})
                    else:
                        #Question time!
                        if not self.question_parse(parsed_list, noname):
                            return None
                else:
                    order_location = match[1]
                               
                    regex = re.compile("(%s)" % '|'.join(self.verbs))
                    order_verb_grp = re.search(regex,match[0])
                    if order_verb_grp != None: order_verb = order_verb_grp.group(0)    

                    print order_location
                    parsed_list.append({"verb" : order_verb, "location" : order_location, "question" : None})
            else:
                #Question time!
                if not self.question_parse(parsed_list, noname):
                    return None

            

        return parsed_list

    def question_parse(self, parsed_list, question):
        regex = re.compile("(%s)" % '|'.join(self.questions))
        question_grp = re.search(regex,question)
        if question_grp != None:
            parsed_list.append({"verb" : None, "location" : None, "question" : question_grp.group(0)})
            return True
        else:
            return False


    def needs_question(self,parsed_order):
        o1 = parsed_order[0]
        o2 = parsed_order[1]
        location = None
        verb = None

        # we have a perfect match, return to sender
        if o2 == o1: return {"diff" : False , "question" : "No difference found", "understood_options": None, "action" : o1,'unknown': None}

        if (not o1['question'] == None and o2['question'] == None) or (not o2['question'] == None and o1['question'] == None):
            #not sure if question or order
            return {"diff" : True , "question" : "could you repeat this part?", "understood_options": None, "action" : None,'unknown': None}
        elif o1['question'] != o2['question'] :
            return {"diff" : True , "question" : "could you repeat your question?", "understood_options": None, "action" : o1,'unknown': 'question'} 
        elif o1['location'] == o2['location'] :
            return {"diff" : False , "question" : "Only difference in verb", "understood_options": None, "action" : o1,'unknown': None}            
        else:
            return {"diff" : True , "question" : "Where did you want me to " + o1['verb'] + "?", "understood_options": None, "action" : o1,'unknown': 'location'}                                

        
        

    def confirm(self, action):
        v = action['action']['verb']
        l = action['action']['location']
        q = action['action']['question']
        confirmation = ""

        if not q == None:
            responses = {\
            'what time is it' : "The current time is: " + time.strftime("%H:%M:%S") + ".",\
            'what is the oldest most widely used drug on earth' : 'The oldest, most widely used drug on earth is coffee.',\
            'who are your creators' : 'My creators are Niels and Sebastiaan.'}
            confirmation = responses[q]
        elif v == "approach":
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

def respond_to_observation(obs):
    nlp = nlp4()
    # actionqueue = []
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
            if(item['unknown'] != None):
                item['action'][item['unknown']] = answer
            else:
                retry = {'2best':[0,0]}
                retry['2best'][0] = answer
                retry['2best'][1] = answer
                item = nlp.get_actions(retry)[0]

        # actionqueue.append(item['action'])
        print "[Alice says]",
        if first_action : 
            print "Okay,",    
        else:
            print "then",
        print nlp.confirm(item)
        first_action = False

    # print "\nactionqueue:"
    # last_object = None
    # for item in actionqueue:
    #     print item

if __name__ == '__main__':
    #result = comm.analyze2best('alice get cup from table one and empty it in bowl on table two',        'alice get cup from table one and empty it in bowl on table one')
    #print comm.resultResponse(result)
    #print comm.parseShortSentence('alice get ccould noup from table one and empty basket on table two')
    testcase = []
    testcase.append({'2best' : ["alice navigate to the living room", "go to the kitchen"]})
    testcase.append({'2best' : ["what is the oldest most widely used drug on earth", "alice what is the oldest most widely used drug on earth"]})
    testcase.append({'2best': ["alice what time is it","alice who are your creators"]})

    for obs in testcase:
        respond_to_observation(obs)
        

