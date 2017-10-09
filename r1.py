			iLane=0
			for lane in lanes:
				vehicles=traci.lane.getLastStepVehicleIDs(lane)
				print(lane, iLane)
				if len(vehicles)==0:
					jam[iLane]=[]
				else:
					vehicles.reverse() # make the vehicle at stop line to be the first element with index=0 in list vehicles
					if len(jam[iLane])==0:
						for v in vehicles:
							speed=traci.vehicle.getSpeed(v)
							flag =0
							if speed < 0.1:
								flag =1
							info=[v,flag,step]
							jam[iLane].append(info)
					else:
						# ?????????? remove jam elements not in list vehicles and update those in list vehicles
						
						# find the index of vehicles[0] in jam list
						vindex=0
						v=vehicles[0]
						for vinfo in jam[iLane]:
							if vinfo[0]!=v:
								vindex+=1
							else:
								break
						print('iLane', iLane,'vindex',vindex)
								
						# remove vehicles whose index is less than vindex
						if vindex!=0:
							for i in range(vindex):
								jam[iLane].pop(i)
								
						jam_num=len(jam[iLane]) # the number of left elements in list jam[iLane]
						v_num=len(vehicles) # the number of vehicles in list vehicles  v_num >= jam_num
						
						print('jam_num', jam_num, 'v_num',v_num)
						
						# update info. of first jam_num vehilces in list vehicles
						for i in range(jam_num):
							if jam[iLane][i][1]==1: # flag==1
								pass
							else:
								v=vehicles[i]
								speed=traci.vehicle.getSpeed(v)
								if speed < 0.1:
									jam[iLane][i][1]=1    # set flag=1
									jam[iLane][i][2]=step  # record the time speed is less than 0.1 m/s
									
						# add info. of the rest vehicles in list vehicles to list jam[iLane]
						rest_num=v_num-jam_num
						if rest_num!=0:
							for i in range(rest_num):
								j=i+jam_num # correcting index
								v=vehicles[j]
								speed=traci.vehicle.getSpeed(v)
								flag =0
								if speed < 0.1:
									flag =1
								info=[v,flag,step]
								jam[iLane].append(info)
							
				iLane=iLane+1 # index of lanes list, process next lane
			
			
			print('jam',jam[0])
			print('id', traci.lane.getLastStepVehicleIDs('1i_0'))
