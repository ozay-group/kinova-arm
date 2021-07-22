# additional_functions.py
from transition_system import transition_system

def find_terminal_states( ts ):
	#Description:
	#	Searches through the states of the given transition system for a terminal state.
	#Outputs:
	#	contains_terminal: Boolean variable that describes whether or not the transition system
	#						has a terminal state.
	#	term_states: a list containing any terminal states.

	contains_terminal = False
	term_states = []
	for state in ts.S:
		#Apply the Post operator to every state.
		temp_post = ts.post([state])

		if not temp_post:
			contains_terminal = True
			term_states.append(state)

	return contains_terminal, term_states

def interleave( ts1 , ts2 ):
	#Description:
	#	Define the interleaved transition system of ts1 and ts2.

	#1. Calculate the new state space (the cartesian product of the old ones).
	S_i = cart_prod(ts1.S,ts2.S)
	
	#2. The new action space is the union of all previous action spaces.
	Act_i = ts1.Act + ts2.Act

	#3. The new initial state set is the cartesion product of the old ones.
	I_i = cart_prod(ts1.I,ts2.I)

	#4. The atomic propositions of this transition system is the union of the 2 previous proposition sets.
	AP_i = ts1.AP + ts2.AP

	#5. The Transition Relation needs to be defined carefully.
	Delta_i = []
	for transition in ts1.trans:
		s = transition[0]
		alpha = transition[1]
		s_p = transition[2]
		for state_i in S_i:
			if state_i[0] is transition[0]:
				Delta_i.append( [ state_i , alpha , (s_p,state_i[1]) ] )

	for transition in ts2.trans:
		s = transition[0]
		alpha = transition[1]
		s_p = transition[2]
		for state_i in S_i:
			if state_i[0] is transition[0]:
				Delta_i.append( [ state_i , alpha , (state_i[0],s_p) ] )
	
	#6. Labelling Function.
	L_i = {}
	for state_i in S_i:
		#Each label will be the union of the individual labels applied to each state.
		L_i[state_i] = list(set(ts1.L[state_i[0]]) | set(ts2.L[state_i[1]])) 


	return transition_system( S_i , Act_i , Delta_i , I_i , AP_i , L_i )

"""
Helper Functions
"""

def cart_prod( set1 , set2 ):
	temp_cp = [];
	for item1 in set1:
		for item2 in set2:
			temp_cp.append( (item1,item2) )

	return temp_cp