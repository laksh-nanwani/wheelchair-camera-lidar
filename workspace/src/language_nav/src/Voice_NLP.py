#!/usr/bin/env python3

from dis import Instruction
from http import client
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
# stanza.install_corenlp()
# stanza.download_corenlp_models(model='english', version='4.2.2')
from stanza.server import CoreNLPClient
from std_msgs.msg import String,Bool
global command
command="start"

def reach_callback(msg):
    global reach
    reach=msg.data

def callback(message):
    global command
    command=message.data

class LangNav():

    global command,reach
    print(command)

    def __init__(self):
        global name_pub,reach
        reach=False
        rospy.init_node("lang_nav")
        
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # self.client.wait_for_server()

        rospy.Subscriber("chatter", String, callback)
        rospy.Subscriber("reached", Bool, reach_callback)

        name_pub=rospy.Publisher("/human_name",String,queue_size=10)
        # self.lookup_table = {
		# 	"table 1": [3.495, -0.686, 0.000, 0.000, 0.000, -0.011, 1.000],        # Rishabh's table
        #     		"table 2": [5.821, -0.548, 0.000, 0.000, 0.000, 0.026, 1.000],	   # Bhanu's table
		# 	"table 3": [5.034, 0.511, 0.000, 0.000, 0.000, 0.701, 0.713],		   #
        #     		"table 4": [6.913, 0.342, 0.000, 0.000, 0.000, -0.018, 1.000],	   # Prathamesh's table
		# 	"table 5": [8.941, 0.197, 0.000, 0.000, 0.000, -0.016, 1.000],  		   # Khushagra's table
		# 	"table 6": [8.065, -1.000, 0.000, 0.000, 0.000, 0.007, 1.000],		   # Jaskirat's table
		# 	"table 7": [10.257, -0.876, 0.000, 0.000, 0.000, -0.719, 0.695],      # Dhruv's table
		# 	"table 8": [14.965, -1.441, 0.000, 0.000, 0.000, -0.191, 0.981],   	   # Laksh's table
		# 	"gate 1" : [13.221, -1.887, 0.000, 0.000, 0.000, -0.696, 0.718],	   # Main door
		# 	"gate 2" : [0.370, -1.624, 0.000, 0.000, 0.000, -0.909, 0.418],	   # Sim door
        # }

        # self.lookup_table = {
		# 	"table 1": [-3.459, -1.208, 0.000, 0.000, 0.000, -0.711, 0.703],        # Dhruvs's table
        #     "table 2": [0.079, 0.240, 0.000, 0.000, 0.000, 0.604, 0.797],	   # Rishabh's table
		# 	"table 3": [-1.355, 0.102, 0.000, 0.000, 0.000, 0.639, 0.769],		   # Bhanu's table
        #     "table 4": [-2.489, 0.754, 0.000, 0.000, 0.000, 0.677, 0.736],	   # Neel's table
		# 	"table 5": [-4.087, 0.528, 0.000, 0.000, 0.000, 0.686, 0.727],  		   # Jaskirat's table
		# 	"table 6": [-5.858, 0.603, 0.000, 0.000, 0.000, 0.704, 0.710],		   # Parth's table
		# 	"table 7": [-11.044, 0.515, 0.000, 0.000, 0.000, 0.994, 0.106],      # Laksh's table
		# 	"table 8": [-9.352, -1.739, 0.000, 0.000, 0.000, -0.988, 0.154],   	   # Intern extra tables
        #     "table 9" :[-5.515, -1.128, 0.000, 0.000, 0.000, -0.696, 0.718],   # Damodar's table
		# 	"gate 1" : [-9.024, 1.293, 0.000, 0.000, 0.000, 0.722, 0.692],	   # Main door
		# 	"gate 2" : [0.426, -0.651, 0.000, 0.000, 0.000, -0.006, 1.000],	   # Partition door
        # }

        # self.lookup_table = {
		# 	"table 1": [-3.233, -1.147, 0.000, 0.000, 0.000, -0.711, 0.703],        # Dhruvs's table
        #     "table 2": [-5.297, -1.013, 0.000, 0.000, 0.000, 0.604, 0.797],	   # Parth's table
		# 	"table 3": [-0.639,  0.197, 0.000, 0.000, 0.000, 0.639, 0.769],		   # Bhanu's table
        #     "table 4": [-2.104,  0.015, 0.000, 0.000, 0.000, 0.677, 0.736],	   # Neel's table
		# 	"table 5": [-5.735,  0.421, 0.000, 0.000, 0.000, 0.686, 0.727],  		   # Jaskirat's table
		# 	"table 6": [-5.858, 0.603, 0.000, 0.000, 0.000, 0.704, 0.710],		   # Parth's table
		# 	"table 7": [-11.044, 0.515, 0.000, 0.000, 0.000, 0.994, 0.106],      # Laksh's table
		# 	"table 8": [-9.352, -1.739, 0.000, 0.000, 0.000, -0.988, 0.154],   	   # Intern extra tables
        #     "table 9" :[-5.515, -1.128, 0.000, 0.000, 0.000, -0.696, 0.718],   # Damodar's table
		# 	"gate 1" : [-9.024, 1.293, 0.000, 0.000, 0.000, 0.722, 0.692],	   # Main door
		# 	"gate 2" : [0.426, -0.651, 0.000, 0.000, 0.000, -0.006, 1.000],	   # Partition door
        # }

        self.lookup_table = {
			"table 1": [9.231, 0.387, 0.000, 0.000, 0.000, 0.103, 0.995],        # Rishabs's table
            "table 2": [7.637, 0.420, 0.000, 0.000, 0.000, 0.015, 1.000],	   # Bhanu's table
			"table 3": [6.591, 0.892, 0.000, 0.000, 0.000, -0.326, 0.945],		   # Neel's table
            "table 4": [4.584, 0.970, 0.000, 0.000, 0.000, -0.034, 0.999],	   # Jaskirat's table
			"table 5": [2.809, 0.952, 0.000, 0.000, 0.000, -0.008, 1.000],  		   # Dhruv's table
			"table 6": [7.944, -1.451, 0.000, 0.000, 0.000, -0.221, 0.975],		   # Sanket's table
			"table 7": [5.097, -0.881, 0.000, 0.000, 0.000, -0.023, 1.000],      # Parth's table
			"table 8": [3.287, -0.855, 0.000, 0.000, 0.000, 0.015, 1.000],   	   # Damodar's tables
            "table 9" :[-0.638, -0.284, 0.000, 0.000, 0.000, -0.977, 0.211],   # Intern extra tables
			"gate 1" : [0.032, 1.489, 0.000, 0.000, 0.000, 0.725, 0.689],	   # Main door
			"gate 2" : [9.587, -0.393, 0.000, 0.000, 0.000, 0.058, 0.998],	   # Partition door
        }
    
    def detect_location(self, user_input):
        detected_nouns = []


        with CoreNLPClient(
                annotators=['tokenize','pos','parse',],
                timeout=30000,
                memory='6G') as client:

            ann = client.annotate(user_input.lower())
            sentence = ann.sentence[0]
            constituency_parse = sentence.parseTree
            # print(constituency_parse)

            for sent in ann.sentence:
                detected_nouns_sent = []
                detected_noun_position=[]
                for token in sent.token:
                    print(token)
                    if(token.pos == "NNP" or token.pos == "NN" or token.pos == "NNPS" or token.pos == "NNS" or token.pos == "CD" or token.pos == "POS"):
                        detected_nouns_sent.append(token.word)
                        detected_noun_position.append(token.beginIndex)              
                if(len(detected_nouns_sent)):
                    detected_nouns=detected_nouns_sent
        

        detected_locations = []
        print(detected_nouns)
        print(detected_noun_position)
        i=0
        while i <len(detected_nouns):
           if i+1<len(detected_nouns):
            if detected_noun_position[i]+1==detected_noun_position[i+1]:
                if detected_nouns[i+1]=="\'s":
                    
                    detected_nouns[i+1]=detected_nouns[i]+ detected_nouns[i+1]
                    detected_nouns.pop(i)
                    detected_noun_position.pop(i)
                else:
                    detected_nouns[i+1]=detected_nouns[i]+" " +detected_nouns[i+1]
                    detected_nouns.pop(i)
                    detected_noun_position.pop(i)   
                print(detected_noun_position)  
                i=0    
           i=i+1    
        print(detected_nouns)        

        for detected_noun_phrase in detected_nouns:
            for i in range(len(detected_noun_phrase)):
                if detected_noun_phrase[i:] in self.lookup_table:
                    detected_locations.append(detected_noun_phrase[i:])
                    break
                    
        
        print(detected_locations)
        return detected_locations
    

    def execute_command(self, user_input):
        global name_pub,reach

        detected_locations = self.detect_location(user_input)

        if(len(detected_locations)):
            for location in detected_locations:
                if location in self.lookup_table.keys():
                    print(f"Going to location: {location}")


                    if location == "gate":
                        
                        target_pose = self.lookup_table[location]
                        goal = MoveBaseGoal()
                        goal.target_pose.header.frame_id = "map"
                        goal.target_pose.header.stamp = rospy.Time.now()
                        goal.target_pose.pose.position.x = target_pose[0]
                        goal.target_pose.pose.position.y = target_pose[1]
                        goal.target_pose.pose.orientation.x = target_pose[2]
                        goal.target_pose.pose.orientation.y = target_pose[3]
                        goal.target_pose.pose.orientation.z = target_pose[4]
                        goal.target_pose.pose.orientation.w = target_pose[5]
                        
                        self.client.send_goal(goal)
                        wait = self.client.wait_for_result()

                        if not wait:
                            rospy.logerr("Action server not available!")
                            rospy.signal_shutdown("Action server not available!")
                        else:
                            if self.client.get_result():
                                target_pose = self.lookup_table[location]
                                goal = MoveBaseGoal()
                                goal.target_pose.header.frame_id = "map"
                                goal.target_pose.header.stamp = rospy.Time.now()
                                goal.target_pose.pose.position.x = 12.0
                                goal.target_pose.pose.position.y = -3.81
                                goal.target_pose.pose.orientation.x = 0
                                goal.target_pose.pose.orientation.y = 0
                                goal.target_pose.pose.orientation.z = -0.711
                                goal.target_pose.pose.orientation.w = 0.702
                                
                                self.client.send_goal(goal)
                                wait = self.client.wait_for_result()

                                if not wait:
                                    rospy.logerr("Action server not available!")
                                    rospy.signal_shutdown("Action server not available!")
                                else:
                                    if self.client.get_result():
                                        
                                        target_pose = self.lookup_table[location]
                                        goal = MoveBaseGoal()
                                        goal.target_pose.header.frame_id = "map"
                                        goal.target_pose.header.stamp = rospy.Time.now()
                                        goal.target_pose.pose.position.x = target_pose[0]
                                        goal.target_pose.pose.position.y = target_pose[1]
                                        goal.target_pose.pose.orientation.x = 0.0
                                        goal.target_pose.pose.orientation.y = 0.0
                                        goal.target_pose.pose.orientation.z = 1.0
                                        goal.target_pose.pose.orientation.w = 0.0
                                        
                                        self.client.send_goal(goal)
                                        wait = self.client.wait_for_result()

                                        if not wait:
                                            rospy.logerr("Action server not available!")
                                            rospy.signal_shutdown("Action server not available!")
                                        else:
                                            if self.client.get_result():
                                                print(f"Reached Location: {location}")
                                            else:
                                                print(f"Couldn't Reach Location: {location}")    
                                    else:
                                        print(f"Couldn't Reach Location: {location}")
                            else:
                                print(f"Couldn't Reach Location: {location}")

                    elif location=="aman":
                        while not reach:
                            name_pub.publish("aman")
                        reach=False    
                    else:
                        target_pose = self.lookup_table[location]
                        goal = MoveBaseGoal()
                        goal.target_pose.header.frame_id = "map"
                        goal.target_pose.header.stamp = rospy.Time.now()
                        goal.target_pose.pose.position.x = target_pose[0]
                        goal.target_pose.pose.position.y = target_pose[1]
                        goal.target_pose.pose.orientation.x = target_pose[2]
                        goal.target_pose.pose.orientation.y = target_pose[3]
                        goal.target_pose.pose.orientation.z = target_pose[4]
                        goal.target_pose.pose.orientation.w = target_pose[5]
                        
                        self.client.send_goal(goal)
                        wait = self.client.wait_for_result()

                        if not wait:
                            rospy.logerr("Action server not available!")
                            rospy.signal_shutdown("Action server not available!")
                        else:
                            if self.client.get_result():
                                print(f"Reached Location: {location}")
                            else:
                                print(f"Couldn't Reach Location: {location}")
                        

                else:
                    print(f"Invalid Location: {location}")
        else:
            print("No Location Detected!!")


    def run(self):
        global command
        print(command)

        while True:
            user_input = command
            print(command)
            
            if user_input != '0':
                self.execute_command(user_input)

            else:
                break


if __name__ == '__main__':
    global name_pub
    
    langNav = LangNav()
    langNav.run()
