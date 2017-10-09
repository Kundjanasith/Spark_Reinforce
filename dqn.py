import random
import numpy
from keras.models import Model, model_from_config
from keras.layers import Convolution2D, Dense, Flatten, Input, merge
from keras.optimizers import RMSprop
from keras import backend as K
from theano import printing
from theano.gradient import disconnected_grad

    
    
class Agent:
    def __init__(self, presence_state_size=None, speed_state_size=None,
				 phase_vector_size=None, number_of_actions=1,
                 epsilon=0.1, mbsz=32, discount=0.9, memory=50,
                 save_name='basic', save_freq=10, w_copy_freq=1,tau=0.001):
        self.presence_state_size = presence_state_size
        self.speed_state_size=speed_state_size
        self.phase_vector_size=phase_vector_size
        self.number_of_actions = number_of_actions
        self.epsilon = epsilon
        self.mbsz = mbsz     #minibatch size
        self.discount = discount
        self.memory = memory # store data of some episodes in memory for experience replay
        self.save_name = save_name #save learned model weight
        self.presence_states = []
        self.speed_states = []
        self.phase_states = []
        self.actions = []
        self.rewards = []
        self.experience = []
        self.i = 1  # count the number of saving model weight
        self.save_freq = save_freq
        self.w_copy_freq=w_copy_freq
        self.w_copy_timer=1
        self.tau=tau
        self.build_functions()


    def build_model(self):
        S1 = Input(shape=self.presence_state_size)
        h1 = Convolution2D(16, 4, 4, subsample=(2, 2),
            border_mode='same', activation='relu')(S1)
        h1 = Convolution2D(32, 2, 2, subsample=(1, 1),
            border_mode='same', activation='relu')(h1)
        S2 = Input(shape=self.speed_state_size)
        h2 = Convolution2D(16, 4, 4, subsample=(2, 2),
            border_mode='same', activation='relu')(S2)
        h2 = Convolution2D(32, 2, 2, subsample=(1, 1),
            border_mode='same', activation='relu')(h2)
        
        S3 = Input(shape=self.phase_vector_size)
        h1 = Flatten()(h1)
        h2 = Flatten()(h2)
        h=merge([h1, h2, S3], mode='concat')
        
        h = Dense(128, activation='relu')(h)
        h = Dense(64, activation='relu')(h)
        V = Dense(self.number_of_actions)(h)
       
        self.model = Model(input=[S1,S2,S3], output=V)
        self.targetDQN = Model(input=[S1,S2,S3], output=V)
        
        try:
            self.model.load_weights('{}.h5'.format(self.save_name+'_model'))
            print "loading model from {}.h5".format(self.save_name+'_model')
            self.targetDQN.load_weights('{}.h5'.format(self.save_name+'_targetDQN'))
            print "loading targetDQN from {}.h5".format(self.save_name+'_targetDQN')
	    
        except:
			self.targetDQN.set_weights(self.model.get_weights())
			print "Training a new model and set target with same weights of model"
            
        
            
    def print_model(self, model):
		i=0
		for layer in model.layers:
			weights = layer.get_weights()
			print "layer ",i,": ",weights
			i += 1

    def build_functions(self):
        S1 = Input(shape=self.presence_state_size)   #one state data, here a state is the presence_array of vehicles
        S2 = Input(shape=self.speed_state_size)
        S3 = Input(shape=self.phase_vector_size)
        NS1 = Input(shape=self.presence_state_size)  # next state data
        NS2 = Input(shape=self.speed_state_size)
        NS3 = Input(shape=self.phase_vector_size)
        
        A = Input(shape=(1,), dtype='int32') #action
        R = Input(shape=(1,), dtype='float32') #reward
        T = Input(shape=(1,), dtype='int32') # indicate if one state is the terminal state in one episode,
                                             # used below to calculate output estimates, see the algorithm part of "nature paper"
        self.build_model()
        
        self.value_fn = K.function([S1,S2,S3], self.model([S1,S2,S3])) # return a list of action values [[v1, v2, ..., vn]]
		
        print('this is with soft target network!!!!!')
        VS = self.model([S1,S2,S3])      # for given state S, calculate action values VS
        #VNS = disconnected_grad(self.model([NS1,NS2,NS3]))  # for next state NS, calculate action values VNS
        VNS = disconnected_grad(self.targetDQN([NS1,NS2,NS3]))
        future_value = (1-T) * VNS.max(axis=1, keepdims=True)  # estimate the maximum action value 
        discounted_future_value = self.discount * future_value #discounting 
        target = R + discounted_future_value    # estimate the target action value
        cost = ((VS[:, A] - target)**2).mean()  # calculate estimation error, the mean square error MSE
        opt = RMSprop(0.0002)
        params = self.model.trainable_weights
        updates = opt.get_updates(params, [], cost)  #update weights/parameters
        self.train_fn = K.function([S1,S2,S3, NS1,NS2,NS3, A, R, T], cost, updates=updates)

    def new_episode(self):
        self.presence_states.append([]) # states=[[], [], [], [] ... []]
        self.speed_states.append([])
        self.phase_states.append([])
        self.actions.append([]) # actions=[[], [], ... []]
        self.rewards.append([]) # rewards=[[], [], ...[]]
        self.presence_states = self.presence_states[-self.memory:]  # keep 'memory' number of episodes data
        self.speed_states = self.speed_states[-self.memory:]
        self.phase_states = self.phase_states[-self.memory:]
        self.actions = self.actions[-self.memory:]
        self.rewards = self.rewards[-self.memory:]

        #self.epsilon=max(1.0-(self.i)*1.0/1000,0.1) # 
        #self.epsilon=max(1.0-(self.i-1)*1.0/500,0)
        #self.epsilon-=(0.1-0.001)/500
        print('\n')
        print('episode NO: ', self.i)
        print('epsilon : ', self.epsilon)
        
        self.i += 1
        
        if self.i % self.save_freq == 0:
            self.model.save_weights('{}.h5'.format(self.save_name+'_model'), overwrite=True)
            self.targetDQN.save_weights('{}.h5'.format(self.save_name+'_targetDQN'), overwrite=True)

    def end_episode(self):
        pass

    def act(self, presence_state, speed_state, phase_state):
		self.presence_states[-1].append(presence_state)
		self.speed_states[-1].append(speed_state)
		self.phase_states[-1].append(phase_state)
		values = self.value_fn([presence_state[None, :],speed_state[None, :],phase_state[None,:]])
        #values = self.value_fn([presence_state[None, :]])

		if numpy.random.random() < self.epsilon:
			action = numpy.random.randint(self.number_of_actions)
		else:
			action = values.argmax()
		self.actions[-1].append(action)
		return action, values
	
    def update_targetDQN(self):
    	model_weights=self.model.get_weights()
    	targetDQN_weights=self.targetDQN.get_weights()
    	for i in xrange(len(model_weights)):
    		targetDQN_weights[i] = self.tau * model_weights[i] + (1 - self.tau)* targetDQN_weights[i]
    		self.targetDQN.set_weights(targetDQN_weights)

		
    def observe(self, reward):
		self.rewards[-1].append(reward)

		return self.iterate()

			

    def iterate(self):
        N = len(self.presence_states) #the number of episodes stored in memory
        S1 = numpy.zeros((self.mbsz,) + self.presence_state_size)
        S2 = numpy.zeros((self.mbsz,) + self.speed_state_size)
        S3 = numpy.zeros((self.mbsz,) + self.phase_vector_size)
        NS1 = numpy.zeros((self.mbsz,) + self.presence_state_size)
        NS2 = numpy.zeros((self.mbsz,) + self.speed_state_size)
        NS3 = numpy.zeros((self.mbsz,) + self.phase_vector_size)
        
        A = numpy.zeros((self.mbsz, 1), dtype=numpy.int32)
        R = numpy.zeros((self.mbsz, 1), dtype=numpy.float32)
        T = numpy.zeros((self.mbsz, 1), dtype=numpy.int32)
        for i in xrange(self.mbsz):
            episode = random.randint(max(0, N-self.memory), N-1)  # we keep only 50 episodes data
            num_frames = len(self.rewards[episode]) 
            # or  num_frames = len(self.presence_states[episode]) 
            frame = random.randint(0, num_frames-1)
            #print('total episodes:',N,'episode:',episode, 'total states',num_frames,'state no:',frame,'total actions',len(self.actions[episode]),'total rewads',len(self.rewards[episode]))
            S1[i] = self.presence_states[episode][frame]
            S2[i] = self.speed_states[episode][frame]
            S3[i] = self.phase_states[episode][frame]
            T[i] = 1 if frame == num_frames - 1 else 0
            if frame < num_frames - 1:
                NS1[i] = self.presence_states[episode][frame+1]
                NS2[i] = self.speed_states[episode][frame+1]
                NS3[i] = self.phase_states[episode][frame+1]
            A[i] = self.actions[episode][frame]
            R[i] = self.rewards[episode][frame]
        cost = self.train_fn([S1,S2,S3, NS1,NS2,NS3, A, R, T])
        self.update_targetDQN()
        #w_DQN=self.model.get_weights()
        #self.targetDQN.set_weights(w_DQN)
        #copy weights to taret DQN
        #print('copy timer: ', self.w_copy_timer)
        #if self.w_copy_timer == self.w_copy_freq:
		#	self.targetDQN.set_weights(self.model.get_weights())
		#	self.w_copy_timer=1
			#print('copy weights to DQN')
        #else:
        #	self.w_copy_timer+=1

        return cost





















