#!/usr/bin/env python

"""

@file    runner.py

@author  Lena Kalleske

@author  Daniel Krajzewicz

@author  Michael Behrisch

@author  Jakob Erdmann

@date    2009-03-26

@version $Id: runner.py 22608 2017-01-17 06:28:54Z behrisch $



Tutorial for traffic light control via the TraCI interface.



SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/

Copyright (C) 2009-2017 DLR/TS, Germany



This file is part of SUMO.

SUMO is free software; you can redistribute it and/or modify

it under the terms of the GNU General Public License as published by

the Free Software Foundation; either version 3 of the License, or

(at your option) any later version.

"""

from __future__ import absolute_import

from __future__ import print_function



import os

import sys

import optparse

import subprocess

import random
import numpy as np
from dqn import Agent
import time
import math


# we need to import python modules from the $SUMO_HOME/tools directory
SUMO_HOME="/usr/share/sumo/"

try:

    sys.path.append(os.path.join(os.path.dirname(

        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests

    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(

        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs

    from sumolib import checkBinary

except ImportError:

    sys.exit(

        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")



import traci





def generate_routefile(rho):

    random.seed(time.time())  # make tests reproducible
    #random.seed(42)  # make tests reproducible

    N = 5400  # number of time steps

    # demand per second from different directions

    pWE = rho*1. /5
    pWEN= rho*1. / 20
    #pWES=1./20
	
    pEW = rho*1. / 5
    pEWS= rho*1. / 20
	
    pNS = rho*1. / 10
    pNSE= rho*1. / 20
    
    pSN = rho*1. / 10
    pSNW= rho*1. / 20
	
    with open("cross.rou.xml", "w") as routes:

        print("""<routes>

        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="19.444" guiShape="passenger"/>

        <vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="25" guiShape="bus"/>



        <route id="WE" edges="51o 1i 2o 52i" />
        <route id="WEN" edges="51o 1i 4o 54i" />
        <route id="WES" edges="51o 1i 3o 53i"/>
        
        <route id="EW" edges="52o 2i 1o 51i" />
        <route id="EWS" edges="52o 2i 3o 53i" />
        
        <route id="NS" edges="54o 4i 3o 53i" />
        <route id="NSE" edges="54o 4i 2o 52i" />
        
        <route id="SN" edges="53o 3i 4o 54i" />
        <route id="SNW" edges="53o 3i 1o 51i" />""", file=routes)

        lastVeh = 0

        vehNr = 0

        for i in range(N):

            if random.uniform(0, 1) < pWE:

                print('    <vehicle id="WE_%i" type="typeWE" route="WE" depart="%i" departLane="random"/>' % (

                    vehNr, i), file=routes)

                vehNr += 1

                lastVeh = i
            if random.uniform(0, 1) < pWEN:

                print('    <vehicle id="WEN_%i" type="typeWE" route="WEN" depart="%i" departLane="random"/>' % (

                    vehNr, i), file=routes)

                vehNr += 1

                lastVeh = i

                
                
            if random.uniform(0, 1) < pEW:

                print('    <vehicle id="EW_%i" type="typeWE" route="EW" depart="%i" departLane="random"/>' % (

                    vehNr, i), file=routes)

                vehNr += 1

                lastVeh = i
            if random.uniform(0, 1) < pEWS:

                print('    <vehicle id="EWS_%i" type="typeWE" route="EWS" depart="%i" departLane="random"/>' % (

                    vehNr, i), file=routes)

                vehNr += 1

                lastVeh = i
                
                
            if random.uniform(0, 1) < pNS:

                print('    <vehicle id="NS_%i" type="typeWE" route="NS" depart="%i" departLane="random" color="1,0,0"/>' % (

                    vehNr, i), file=routes)

                vehNr += 1

                lastVeh = i
            if random.uniform(0, 1) < pNSE:

                print('    <vehicle id="NSE_%i" type="typeWE" route="NSE" depart="%i" departLane="random" color="1,0,0"/>' % (

                    vehNr, i), file=routes)

                vehNr += 1

                lastVeh = i
                
            if random.uniform(0, 1) < pSN:

                print('    <vehicle id="SN_%i" type="typeWE" route="SN" depart="%i" departLane="random" color="1,0,0"/>' % (

                    vehNr, i), file=routes)

                vehNr += 1

                lastVeh = i
            if random.uniform(0, 1) < pSNW:

                print('    <vehicle id="SNW_%i" type="typeWE" route="SNW" depart="%i" departLane="random" color="1,0,0"/>' % (

                    vehNr, i), file=routes)

                vehNr += 1

                lastVeh = i
                
        print("</routes>", file=routes)




def get_options():

    optParser = optparse.OptionParser()

    optParser.add_option("--nogui", action="store_true",

                         default=False, help="run the commandline version of sumo")

    options, args = optParser.parse_args()

    return options

#f_path="data/compare(rho=0.1-1)/" 


f_path="data/test/"
if not os.path.exists(f_path):
	os.makedirs(f_path)

f_staytime = open(f_path+"staytimeallroads.txt", "wb")
f_vnum = open(f_path+"vnumallroads.txt", "wb")
f_vnum_road0 = open(f_path+"vnumroad0.txt", "wb")
f_vnum_road1 = open(f_path+"vnumroad1.txt", "wb")
f_vnum_road2 = open(f_path+"vnumroad2.txt", "wb")
f_vnum_road3 = open(f_path+"vnumroad3.txt", "wb")
f_delay = open(f_path+"delayallroads.txt", "wb")
f_delay_road0 = open(f_path+"delayroad0.txt", "wb")
f_delay_road1 = open(f_path+"delayroad1.txt", "wb")
f_delay_road2 = open(f_path+"delayroad2.txt", "wb")
f_delay_road3 = open(f_path+"delayroad3.txt", "wb")



# this is the main entry point of this script

if __name__ == "__main__":

    options = get_options()



    # this script has been called from the command line. It will start sumo as a

    # server, then connect and run

    #if options.nogui:

    #    sumoBinary = checkBinary('sumo')

    #else:

    #    sumoBinary = checkBinary('sumo-gui')
    
    
    #sumoBinary = checkBinary('sumo')
    sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    phase_num=2
    lane_num = 16
    lane_len=500 # meters
    cell_num=20
    car_len=5 # meters
    car_gap=2.5 # meters
    cell_len=8
    cells_len=cell_num*cell_len
    speed_limit=19.444 # m/s
    presence_array=np.zeros((lane_num,cell_num),dtype=np.int)
    speed_array=np.zeros((lane_num,cell_num),dtype=np.float)
    phase_vector=np.zeros(phase_num,dtype=np.int)
    phase_vector[0]=1 # indicate phase 0 is enacted
    
    
    channel=1
    state=np.zeros((lane_num,cell_num,channel))
    f_name='gao'
    agent = Agent(presence_state_size=state.shape,speed_state_size=state.shape,
			  phase_vector_size=phase_vector.shape,
              number_of_actions=phase_num,epsilon=0.1,discount=0.95,memory=200,
              save_name=f_name,w_copy_freq=10,tau=0.001)
              

    
    rho=0 # parameter controlling vehicle arrival rates
    run_num=101
    
    episode_num=run_num
    print('episode total: ',episode_num)
    for i in range(episode_num):
		
		t0 = time.time()		
		agent.new_episode()
		
		#f_staytime.write('*********** new episode: %d *****************\n' %(i))
		#f_vnum.write('*********** new episode: %d *****************\n' %(i))
		#f_vnum_road0.write('*********** new episode: %d *****************\n' %(i))
		#f_vnum_road1.write('*********** new episode: %d *****************\n' %(i))
		#f_vnum_road2.write('*********** new episode: %d *****************\n' %(i))
		#f_vnum_road3.write('*********** new episode: %d *****************\n' %(i))
		#f_delay.write('*********** new episode: %d *****************\n' %(i))
		#f_delay_road0.write('*********** new episode: %d *****************\n' %(i))
		#f_delay_road1.write('*********** new episode: %d *****************\n' %(i))
		#f_delay_road2.write('*********** new episode: %d *****************\n' %(i))
		#f_delay_road3.write('*********** new episode: %d *****************\n' %(i))
		

		presence_array=np.zeros((lane_num,cell_num),dtype=np.int)
		speed_array=np.zeros((lane_num,cell_num),dtype=np.float)
		phase_vector=np.zeros(phase_num,dtype=np.int)
		phase_vector[0]=1 # indicate phase 0 is enacted
		#state
		presence_observation=presence_array.reshape((lane_num,cell_num,channel))
		speed_observation=speed_array.reshape((lane_num,cell_num,channel))
		phase_obervation=phase_vector
		#reward
		delay_former=0 # cumulative delay of former second
		delay_current=0
		reward =0
		
		
		#rho=0.1+i*(1-0.1)/(run_num-1)
		rho=1.0
		print('rho: ', rho)
		generate_routefile(rho)
		# this is the normal way of using traci. sumo is started as a
		# subprocess and then the python script connects and runs
		traci.start([sumoBinary, "-c", "cross.sumocfg","--tripinfo-output", "tripinfo.xml"])
		
		#run()
		action=0 # two actions: 0 is phase 0, 1 is phase 4
		phase=[0,4]
		previous_action=0
		
		values=[]
		action_duration=10   # duration of actions, it is equal to the queen interval of traffic signals
		transition_duration=18
		select_action=1 #
		transition_phase=0 # 1: if action is changed, then a transition phase follows; else, no transition phase
		observe_reward=0 # 1: observe reward  when timers expires   0: do not observe
		
		timer_len=action_duration
		step = 0
		timer=0 # counts the time to decide the time agent should act
		

		lanes=['1i_0','1i_1','1i_2','1i_3','2i_0','2i_1','2i_2','2i_3','3i_0','3i_1','3i_2','3i_3','4i_0','4i_1','4i_2','4i_3']
		edges=["1i","2i","3i","4i"]
		v_onroads=[[],[],[],[]] # IDs of vehicles on all edges/roads
		enter_time=[[],[],[],[]] # time of entering roads/edges for all vehicles
		#jam=[[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]] # info. of jammed vehicles of four edges, [ID, jam flag, jam beginning time, 
		                  #leaving intersection time],e.g.,[v0,1,23,40]
		v_delay=[[],[],[],[]] # statistic of leaving time - entering time
		v_num=[[],[],[],[]] # number of vehicles staying at roads
		v_staytime=[]
		                  
		sim_len=5400
		
		#important note: set phase and simulationStep() should be paired 
		traci.trafficlights.setPhase("0",0)# initialize phase to 0
		traci.simulationStep() # excecute one round first to align simu time
		while step<sim_len:

			#print('******************** ')

			
			# agent selects action and sets phase
			if select_action==1:
				select_action=0 # finish selecting action, so reset it
				action, values = agent.act(presence_observation,speed_observation,phase_obervation)
				#print('action: ',action,'values: ',values)

				
				if action==previous_action: # action not changed
					transition_phase=0 # not a transition phase
					
					traci.trafficlights.setPhase("0",phase[action])
					#print('set new phase to: ',phase[action])

					timer_len=action_duration
					previous_action=action
					
					phase_vector=np.zeros(phase_num,dtype=np.int)
					phase_vector[action]=1
				else: # action changed
					transition_phase=1 # it is a transition phase
					
					timer_len=transition_duration
					traci.trafficlights.setPhase("0",phase[previous_action]+1)
					#print('transition phase: ',phase[previous_action]+1)

					
					
			
			# excecute simulation
			traci.simulationStep()  #
			
			
			
			step += 1 # important note: step increases only if sim is executed
			timer+=1
			#print('time',step)
			#print('currunt phase: ',traci.trafficlights.getPhase("0"))
			
			# maintain vehicle information
			iEdge=0
			for edge in edges:
				vehicles=traci.edge.getLastStepVehicleIDs(edge)
				v_num[iEdge].append(len(vehicles))  # collect data
				if len(vehicles)==0:
					while len(v_onroads[iEdge])!=0:
						v=v_onroads[iEdge].pop(-1)
						et=enter_time[iEdge].pop(-1)
						v_delay[iEdge].append(step-et) # collect data
						#if iEdge==0:
							#print('********************')
							#print('pop v', v)
							#print('enter_time: ', et)
							#print('leave_time: ',step)
					v_onroads[iEdge]=[]
					enter_time[iEdge]=[]
				else:
					#print('********************')
					#print('vehicles', vehicles)
					# put new vehicles into list v_onroads
					for v in vehicles:
						if v in v_onroads[iEdge]:
							pass
						else:
							v_onroads[iEdge].append(v)
							enter_time[iEdge].append(step)
							
					#print('v_onroads',v_onroads[0])
					#print('enter_timer',enter_time[0])

					
					# remove vehicles leaving intersection from list v_onroads
					to_remove=[] # record index of vehicles to be removed  in v_onroads
					index=0
					for v in v_onroads[iEdge]:
						if v in vehicles:
							pass
						else:
							to_remove.append(index)
						index+=1
					
					#print('iEdge', iEdge)
					#print('len v_onroads',len(v_onroads[iEdge]))
					#print('len enter_timer',len(enter_time[iEdge]))
					#print('to_remove', to_remove)
					#print('v_onroads',v_onroads[0])
					
					# remove
					to_remove.reverse() # must pop element with large index, otherwise error occurs
					for i in to_remove:
						v=v_onroads[iEdge].pop(i)
						et=enter_time[iEdge].pop(i)
						v_delay[iEdge].append(step-et) # collect data
						#if iEdge==0:
							#print('********************')
							#print('to_remove', to_remove)
							#print('pop v', v)
							#print('enter_time: ', et)
							#print('leave_time: ',step)
						
				iEdge+=1	
							
			staytime_sum=0 # the sum of the staying time at the intersection roads of all vehicles
			iEdge=0
			for edge in edges:
				for t in enter_time[iEdge]:
					staytime_sum+=step-t
				iEdge+=1
			v_staytime.append(staytime_sum)
			
			#print('********************')
			#print('step',step)
			#print('v_onroads',v_onroads[0])
			#print('enter_timer',enter_time[0])
			#print('v_num[0]', v_num[0][len(v_num[0])-1])
			#print('v_delay[0]', v_delay[0])
			
			

			
			
			
			#timer expires, decide if we observe rewards or change action
			if timer==timer_len:
				timer=0 # reset timer
				if transition_phase==1: # this is a transition phase, set the next corresponding phase
					transition_phase=0 # transition phase finishes, so reset this indicator
					
					traci.trafficlights.setPhase("0",phase[action]) # set phase to the selected action
					#print('set new phase to: ',phase[action])

					timer_len=action_duration
					previous_action=action
					
					phase_vector=np.zeros(phase_num,dtype=np.int)
					phase_vector[action]=1
					
					# recalculate delay info., delay change due to transition phase should 
					# not be counted, thus we recalculate delay_former info.
					delay_former=staytime_sum
					observe_reward=0 # do not observe reward at this moment
					
				else: # transition_phase==0, this is a common action phase
					observe_reward=1 #  observe reward at this moment
			
			if step==sim_len: # to make the number of stored rewards to be equal to number of actions, we need to do this
				observe_reward=1 #  observe reward at this moment
				
			## observe rewards and backprop
			if observe_reward==1:
				observe_reward=0 # reset 
				select_action=1	 # signaling to selection action
				

				delay_current=staytime_sum
				#print('delay_current from edges: ',delay_current)
				
				reward=delay_former-delay_current
				#print('delay_former: ', delay_former)
				#print('reward',reward)
				delay_former=delay_current
				cost=agent.observe(reward)
				
					
				#presence state
				presence_array=np.zeros((lane_num,cell_num),dtype=np.int)
				speed_array=np.zeros((lane_num,cell_num),dtype=np.float)
				iLane=0
				for lane in lanes:
					vehicles=traci.lane.getLastStepVehicleIDs(lane)
					lane_len=traci.lane.getLength(lane)
					for v in reversed(vehicles):
						pos=lane_len-traci.vehicle.getLanePosition(v)
						if pos >= cells_len:
							break
						else:
							iCell=(int)(math.floor(pos/cell_len))
							presence_array[iLane][iCell]=1
							speed=traci.vehicle.getSpeed(v)
							speed_array[iLane][iCell]=speed/speed_limit 
					iLane+=1 # index of lanes list
				presence_observation=presence_array.reshape((lane_num,cell_num,channel))
				speed_observation=speed_array.reshape((lane_num,cell_num,channel))
				phase_obervation=phase_vector
				#print('presence_array lane 0: ', presence_array[0])
				#print('speed_array lane 0: ',speed_array[0])
				#print('phase_obervation: ',phase_obervation)
			
	
		# record statistic data
		tmp_sum=0
		for j in v_num[0]:
			tmp_sum+=j
		avg=1.0*tmp_sum/len(v_num[0])
		f_vnum_road0.write(str(avg)+'\n')
		print('len v_num[0] ', len(v_num[0]))
		print('avg ', avg)
		
		tmp_sum=0
		for j in v_num[1]:
			tmp_sum+=j
		avg=1.0*tmp_sum/len(v_num[1])
		f_vnum_road2.write(str(avg)+'\n')
		print('len v_num[1] ', len(v_num[1]))
		print('avg ', avg)
		
		tmp_sum=0
		for j in v_num[2]:
			tmp_sum+=j
		avg=1.0*tmp_sum/len(v_num[2])
		f_vnum_road1.write(str(avg)+'\n')
		print('len v_num[2] ', len(v_num[2]))
		print('avg ', avg)
		
		tmp_sum=0
		for j in v_num[3]:
			tmp_sum+=j
		avg=1.0*tmp_sum/len(v_num[3])
		f_vnum_road3.write(str(avg)+'\n')
		print('len v_num[3] ', len(v_num[3]))
		print('avg ', avg)
		

		tmp_sum=0
		for j in v_delay[0]:
			tmp_sum+=j
		avg=1.0*tmp_sum/len(v_delay[0])
		f_delay_road0.write(str(avg)+'\n')
		print('len v_delay[0] ', len(v_delay[0]))
		print('avg ', avg)
		
		tmp_sum=0
		for j in v_delay[1]:
			tmp_sum+=j
		avg=1.0*tmp_sum/len(v_delay[1])
		f_delay_road2.write(str(avg)+'\n')
		print('len v_delay[1] ', len(v_delay[1]))
		print('avg ', avg)
		
		tmp_sum=0
		for j in v_delay[2]:
			tmp_sum+=j
		avg=1.0*tmp_sum/len(v_delay[2])
		f_delay_road1.write(str(avg)+'\n')
		print('len v_delay[2] ', len(v_delay[2]))
		print('avg ', avg)
		
		tmp_sum=0
		for j in v_delay[3]:
			tmp_sum+=j
		avg=1.0*tmp_sum/len(v_delay[3])
		f_delay_road3.write(str(avg)+'\n')
		print('len v_delay[3] ', len(v_delay[3]))
		print('avg ', avg)
		
		tmp_sum=0
		for j in v_staytime:
			tmp_sum+=j
		avg=1.0*tmp_sum/len(v_staytime)
		f_staytime.write(str(avg)+'\n')
		print('len v_staytime ', len(v_staytime))
		print('avg ', avg)
			
		traci.close()
		sys.stdout.flush()
		t1 = time.time()
		print('time consumed: ',t1-t0)
    
    

f_staytime.close()

f_vnum.close()
f_vnum_road0.close()
f_vnum_road1.close()
f_vnum_road2.close()
f_vnum_road3.close()

f_delay.close()
f_delay_road0.close()
f_delay_road1.close()
f_delay_road2.close()
f_delay_road3.close()
