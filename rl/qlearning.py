import numpy as np
import pandas as pd
import time 

np.random.seed(2)

N_STATE=6
ACTIONS=['left', 'right']
EPSILON = 0.9
ALPHA = 0.1
LAMBDA = 0.9
MAX_EPISODES = 13
FRESH_TIME = 0.3

def build_q_table(n_states, actions):
    table= pd.DataFrame(np.zeros((n_states, len(actions))), columns=actions)
    print(table)
    return table

def choose_action(state,  q_table):
    state_actions=q_table.iloc[state, :]
    if (np.random.uniform() > EPSILON)  or (state_actions.all()==0):
        action_name = np.random.choice(ACTIONS)
    else :
        action_name= state_actions.argmax()
    return action_name


def  get_env_feedback(s,a):
    if a == 'right':
        if s== N_STATE-2:
            s_ = 'terminal'
            r=1
        else :
            s_= s+1 
            r=0
    else :
        r=0
        if s==0:
            s_=s
        else :
            s_=s-1
    return s_, r

