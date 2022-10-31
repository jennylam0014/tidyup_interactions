#!/usr/bin/env python
# license removed for brevity
import rospy, sys
from std_msgs.msg import String
import random 
from interaction_functions import * 

sc = SoundClient(blocking=True)
bc = SoundClient()


def talk(text): 
	print('Fetch: ' + text)
	sc.say(text,'voice_us1_mbrola')


def callback(msg):
    global convo_flag
    global counter

    convo_flag = True
    if counter == 0: 
        convo(convo_flag) #makes it only call the function once
    counter += 1 

def listening(ARGS,c_keywords,current_stage):
	#beep = bc.playWave('/home/jenny/base/src/Tidy-Up-My-Room-Fetch/destination/scripts/beep.wav')
	beep = bc.playWave('/home/hrigroup/jake/jsan_ws/src/destination/scripts/beep.wav')

	waiting = True
	global stage_4_second_pass
	while waiting:
		beep
		user_input = listen(ARGS)
		text = user_input.split(" ")

		r = [s for s in text if any(xs in s for xs in c_keywords)]
		if r:
			waiting = False
			if r[0] == 'no':
				if current_stage == 4:
					stage_4_second_pass = True
					break
				else: 
					current_stage = current_stage - 1 
			else:
				current_stage = current_stage + 1 
	return r[0],current_stage

    
def convo(convo_flag):

	rospy.sleep(1)
	num_stages = 6 # this might be hard coded ? 
	current_stage = 0

	colour_human = 'null'
	colour_fetch = 'null'

	while current_stage < num_stages:
		question = ['Would you like to clean together?', #stage 0
					'OK. Which colour would you like me to collect?', #stage 1
					'You would like me to pick up the %s coloured blocks?' %colour_fetch, #stage 2 
					'OK, I''ll pick up the %s blocks.' %colour_fetch, #stage 3
                    'I think you should pick up the %s blocks. What do you think?' %colour_human, #stage 4
					'OK, you can pick up %s blocks. Ready to clean?' %colour_human, #stage 5
					'Lets clean'] #stage 6

		keywords = [["yes", "ok", "good", "sure", "no"], #stage 0
		    		['red','blue','yellow','green'], #stage 1
		    		["yes","no"],
                    [],
				    ["ok","yes", "good", "sure", "no"], 
				    ['yes', 'no'],v
				    []]

		global stage_4_second_pass


		c_question = question[current_stage]
		c_keywords = keywords[current_stage]

		if (current_stage == 0): #stage 0: Binary Y/N for assistance
			talk(c_question)
			reply ,current_stage = listening(ARGS,c_keywords,current_stage) 

		elif (current_stage == 1): #stage 1: Colour delgation to Fetch
			talk(c_question)
			colour_fetch ,current_stage = listening(ARGS,c_keywords,current_stage)

		
		elif (current_stage == 2): #stage 2: Confirmation of Task
			talk(c_question)
			response ,current_stage = listening(ARGS,c_keywords,current_stage)


		elif (current_stage == 3): #stage 3: Colour delgation to Human
			talk(c_question)
			current_stage += 1 
			human_colour_choice = keywords[1]
			human_colour_choice.remove(colour_fetch)
			colour_human = human_colour_choice[random.randint(0,2)]
			stage_4_second_pass = False


		elif (current_stage == 4): #stage 4: Colour delgation to Human
			if stage_4_second_pass == True:
				talk("What coloured blocks did you want to pick up?")
				colour_human, current_stage= listening(ARGS,human_colour_choice, current_stage)
			else: 
				talk(c_question)
				response ,current_stage = listening(ARGS,c_keywords,current_stage)

		elif (current_stage == 5): # stage 5: Ready to clean?
			talk(c_question)
			response ,current_stage = listening(ARGS,c_keywords,current_stage)
		
		elif (current_stage == 6):
			talk(c_question)
			current_stage += 1	
    
	colours =[colour_fetch, colour_human]

	for x in range(0,40):  
		colour_pub.publish(colours)
		convo_flag = False

	sys.exit(0) 



if __name__ == '__main__':
    # Initiate
	rospy.init_node('user_input', anonymous=True, disable_signals=True)
	colour_pub = rospy.Publisher('colour', String, queue_size=10)
	rospy.Subscriber('convo', String, callback)
	print("Waiting for Conversation node to publish.")

	convo_flag = False
	counter = 0
	
	BEAM_WIDTH = 500
	DEFAULT_SAMPLE_RATE = 16000
	LM_ALPHA = 0.75
	LM_BETA = 1.85
    

	import argparse
	global ARGS
	parser = argparse.ArgumentParser(description="Stream from microphone to DeepSpeech using VAD")

	parser.add_argument('-v', '--vad_aggressiveness', type=int, default=3,
                        help="Set aggressiveness of VAD: an integer between 0 and 3, 0 being the least aggressive about filtering out non-speech, 3 the most aggressive. Default: 3")
	parser.add_argument('--nospinner', action='store_true',
                        help="Disable spinner")
	parser.add_argument('-w', '--savewav',
                        help="Save .wav files of utterences to given directory")
	parser.add_argument('-f', '--file',
                        help="Read from .wav file instead of microphone")

	parser.add_argument('-m', '--model', default='/home/jenny/base/src/Tidy-Up-My-Room-Fetch/models/output_graph.pbmm',
                        help="Path to the model (protocol buffer binary file, or entire directory containing all standard-named files for model)") #required=True,
	
	parser.add_argument('-l', '--lm', default='/home/jenny/base/src/Tidy-Up-My-Room-Fetch/models/lm.binary',
                        help="Path to the language model binary file. Default: lm.binary")
	
	parser.add_argument('-t', '--trie', default='/home/jenny/base/src/Tidy-Up-My-Room-Fetch/models/trie',
                        help="Path to the language model trie file created with native_client/generate_trie. Default: trie")
	
	parser.add_argument('-d', '--device', type=int, default=None,
                        help="Device input index (Int) as listed by pyaudio.PyAudio.get_device_info_by_index(). If not provided, falls back to PyAudio.get_default_device().")

	parser.add_argument('-r', '--rate', type=int, default=DEFAULT_SAMPLE_RATE,
                        help="Input device sample rate. Default: {DEFAULT_SAMPLE_RATE}. Your device may require 44100.")

	parser.add_argument('-la', '--lm_alpha', type=float, default=LM_ALPHA,
                        help="The alpha hyperparameter of the CTC decoder. Language Model weight. Default: {LM_ALPHA}")
	parser.add_argument('-lb', '--lm_beta', type=float, default=LM_BETA,
                        help="The beta hyperparameter of the CTC decoder. Word insertion bonus. Default: {LM_BETA}")
	parser.add_argument('-bw', '--beam_width', type=int, default=BEAM_WIDTH,
                        help="Beam width used in the CTC decoder when building candidate transcriptions. Default: {BEAM_WIDTH}")
	
	ARGS, unknown = parser.parse_known_args()

    # Wait for a pick object request
	while not rospy.is_shutdown():
		rospy.Subscriber('convo', String, callback)
		rospy.sleep(0.1)
