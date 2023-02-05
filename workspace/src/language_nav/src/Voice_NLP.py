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
			"table 1": [21.594, -1.615, 0.000, 0.000, 0.000, -0.133, 0.991],        # Laksh's table
            "table 2": [14.054, -1.395, 0.000, 0.000, 0.000, -0.707, 0.708],	        # Jaski's table
			"table 3": [12.217, -1.444, 0.000, 0.000, 0.000, -0.782, 0.623],		    # Neel's table
            "table 4": [10.752, -0.752, 0.000, 0.000, 0.000, -0.697, 0.717],	        # Bhanu's table
			"table 5": [8.808, -0.933, 0.000, 0.000, 0.000, -0.707, 0.707],         # Rishabh's table
			"table 6": [8.938, 1.270, 0.000, 0.000, 0.000, 1.000, 0.028],        #   Equipments table
			"table 7": [13.472, 1.352, 0.000, 0.000, 0.000, 0.707, 0.707],        # Parth's table
			"table 8": [15.446, 1.340, 0.000, 0.000, 0.000, 0.708, 0.706],        # Damodar's table
            "table 9": [20.734, 1.776, 0.000, 0.000, 0.000, 0.701, 0.714],        # Interns' table
            "table 10": [6.45, 0.666, 0.000, 0.000, 0.000, 0.676, 0.737],        # Sanket's table
			"gate 1" : [18.664, -2.658, 0.000, 0.000, 0.000, -0.771, 0.636],	    # Main door
            "gate 2" : [7.875, 0.279, 0.000, 0.000, 0.000, -0.999, 0.051],	    # Back door
            # "gate 3" : [15.463, -5.776, 0.000, 0.000, 0.000, -0.705, 0.709],
            "gate 3" : [15.327, -4.463, 0.000, 0.000, 0.000, -0.685, 0.728],	    # Faculty cabins' door
            "gate 4" : [7.092, -4.063, 0.000, 0.000, 0.000, -0.699, 0.715],	    # Robotics club
            # "gate 4" : [5.092, -4.715, 0.000, 0.000, 0.000, -0.699, 0.715],	    # Robotics club
            "corridor": [12.0, -4.063, 0.000, 0.000, 0.000, -0.699, 0.715], # midpoint between robotics club and faculty door
            "sim room": [5.092, -1.5063, 0.000, 0.000, 0.000, -0.699, 0.715] #Sim room waypoint
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
