/*
concurrentgamemodel.go
Description:
 	Basic implementation of a Concurrent Game Model discussed Game-Theoretic Semantics for Alternating-Time Temporal Logic.
*/
package OzayGroupExploration

type ConcurrentGameModel struct {
	Agents []CGMAgent
	St     []CGMState
	Pi     []AtomicProposition
	Act    []string                           // Actions
	d      map[CGMAgent]map[CGMState][]string //Permitted Actions (Action Function)
	o      map[CGMState]map[string]CGMState   //Transition Function
	v      map[AtomicProposition][]CGMState   // Valuation Function
}

type CGMAgent struct {
	Name       string
	ParentGame *ConcurrentGameModel
}

type CGMState struct {
	Name       string
	ParentGame *ConcurrentGameModel
}

// Atomic Proposition Already Defined

/*
Functions
*/

func CreateConcurrentGameModel(agentNames []string, stateNames []string, atomicPropositionNames []string, actionNames []string, actionFunction map[string]map[string][]string, transitionFunction map[string]map[string]string, valuationFunction map[string][]string) (ConcurrentGameModel, error) {
	//Create Empty CGM To Start
	cgm := ConcurrentGameModel{
		Act: actionNames,
	}

	//Create Each Agent.
	var cgmAgents []CGMAgent
	for _, name := range agentNames {
		cgmAgents = append(cgmAgents, CGMAgent{Name: name, ParentGame: &cgm})
	}
	cgm.Agents = cgmAgents

	//Create Concurrent Game Model States
	var cgmStates []CGMState
	for _, name := range stateNames {
		cgmStates = append(cgmStates, CGMState{Name: name, ParentGame: &cgm})
	}
	cgm.St = cgmStates

	// Create Atomic Propositions
	var cgmAPs []AtomicProposition
	for _, name := range atomicPropositionNames {
		cgmAPs = append(cgmAPs, AtomicProposition{Name: name})
	}
	cgm.Pi = cgmAPs

	// // Create Actions
	// cgm.Act = actionNames

	// Create Permitted Action Sets
	permittedActionMap := make(map[CGMAgent]map[CGMState][]string)
	for tempAgentName, stateMap := range actionFunction {
		tempAgent := CGMAgent{Name: tempAgentName, ParentGame: &cgm}
		tempCGMStateMap := make(map[CGMState][]string)
		for tempStateName, subsetOfActions := range stateMap {
			tempState := CGMState{Name: tempStateName, ParentGame: &cgm}
			tempCGMStateMap[tempState] = subsetOfActions
		}
		permittedActionMap[tempAgent] = tempCGMStateMap
	}
	cgm.d = permittedActionMap

	// Create Transition Function based ON Joint Actions
	tempTransitionFunction := make(map[CGMState]map[string]CGMState)
	for tempStateName, jointActionMap := range transitionFunction {
		tempState := CGMState{Name: tempStateName, ParentGame: &cgm}
		tempJointActionMap := make(map[string]CGMState)
		for jointAction, successorStateName := range jointActionMap {
			tempJointActionMap[jointAction] = CGMState{Name: successorStateName, ParentGame: &cgm}
		}
		tempTransitionFunction[tempState] = tempJointActionMap
	}
	cgm.o = tempTransitionFunction

	// Create Valuation Function
	tempValuationFunction := make(map[AtomicProposition][]CGMState)
	for apName, cgmStateNames := range valuationFunction {
		tempAP := AtomicProposition{Name: apName}
		var tempCGMStates []CGMState
		for _, cgmStateName := range cgmStateNames {
			tempCGMStates = append(tempCGMStates, CGMState{Name: cgmStateName, ParentGame: &cgm})
		}
		tempValuationFunction[tempAP] = tempCGMStates
	}
	cgm.v = tempValuationFunction

	//transitionFunction

	return cgm, nil

}
