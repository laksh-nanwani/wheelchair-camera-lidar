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
        
        self.lookup_table = {
			"table 1": [-2.906, 1.543, 0.000, 0.000, 0.000, 0.974, 0.226],        # Laksh's table
            "table 2": [4.541, 1.283, 0.000, 0.000, 0.000, 0.698, 0.716],	        # Jaski's table
			"table 3": [6.574, 1.303, 0.000, 0.000, 0.000, 0.700, 0.714],		    # Neel's table
            "table 4": [7.955, 0.720, 0.000, 0.000, 0.000, 0.698, 0.716],	        # Bhanu's table
			"table 5": [9.535, 0.816, 0.000, 0.000, 0.000, 0.703, 0.712],         # Rishabh's table
			"table 6": [9.647, -1.790, 0.000, 0.000, 0.000, -0.005, 1.000],        # Equipments table
			"table 7": [5.087, -1.211, 0.000, 0.000, 0.000, -0.673, 0.740],        # Parth's table
			"table 8": [3.183, -1.267, 0.000, 0.000, 0.000, -0.689, 0.724],        # Damodar's table
            "table 9": [-2.462, -1.716, 0.000, 0.000, 0.000, -0.709, 0.705],        # Interns' table
			"gate 1" : [-0.294, 1.914, 0.000, 0.000, 0.000, 0.712, 0.702],	    # Main door
            "gate 2" : [10.497, -0.245, 0.000, 0.000, 0.000, -0.040, 0.999],	    # Back door
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
