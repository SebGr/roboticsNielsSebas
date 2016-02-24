import basebehavior.behaviorimplementation
import time
import nlp4
from JSGFParser import JSGFParser

class MoveToLocation_x(basebehavior.behaviorimplementation.BehaviorImplementation):

    def implementation_init(self):

        print "[behavior] Initializing Example Behavior"
        self.last_recogtime         = time.time()
        self.last_speech_obs        = None  # contains the result and 2best form sphinx
        self.new_speech_obs         = False # is used for triggering the conversation 

    def update_last_speech_command(self):

        # sets the new_command boolean and sets the last understood speech_observation
        if (self.m.n_occurs('voice_command') > 0):
            (recogtime, obs) = self.m.get_last_observation("voice_command")
            if not obs == None and recogtime > self.last_recogtime:
                # print "[observation] = ",obs
                self.last_speech_obs = obs
                self.last_recogtime = recogtime
                self.new_speech_obs = True
            else:
                self.new_speech_obs = False

    def formulate_response(self,question):
        grammar = JSGFParser('speech/hark-sphinx/grammar/NielsSebastiaan.gram')
        language_parsing = nlp()
        question = language_parsing.remove_name(language_parsing.remove_opts(question)).lower()
        question = question.replace("?", "").replace(",", "")
        print question
        if(grammar.findToken(question) != None):
        #Question
            responses = {\
            'what time is it' : "The current time is: " + time.strftime("%H:%M:%S") + ".",\
            'what is the oldest most widely used drug on earth' : 'The oldest, most widely used drug on earth is coffee.',\
            'who are your creators' : 'My creators are Niels and Sebastiaan.'}
            return responses[question]
        else:
            words = question.split(' ')
            if(grammar.findTokenVar(words[0]) == '<verb>'):
                #command
                question = question.replace(words[0] + " to the ", "")
                if(grammar.findTokenVar(question) == '<location>'):
                    return "I am moving to " + question + "."
            elif("dining table" in question):
                #request
                return "I am approaching the dining table."
            else:
                return "I'm sorry, I did not get that. Could you please repeat it?"
                

        #try:
        #    responses = {\
        #    'what time is it' : time.strftime("%H:%M:%S"),\
        #    'what is the oldest most widely used drug on earth' : 'coffee',\
        #    'who are your creators' : 'My creators are Niels and Sebastiaan',\
        #    'approach the dining table' : 'I will approach the dining table',\
        #    'please stop' : 'okay, I will stop'}
        #    return responses[question]
        #except:
        #    return None

    def implementation_update(self): #remember this is a loop in the architecture with ~10Hz execution

        self.update_last_speech_command()
        if self.new_speech_obs:
            nlp4.respond_to_observation(self.last_speech_obs)